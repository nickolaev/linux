// SPDX-License-Identifier: GPL-2.0-only
/*
 * Multikernel PCI assignment policy
 *
 * Keeps assigned-device discovery, identity presentation, bridge traversal,
 * and resource restoration independent from the manifest that populated
 * the current instance.
 */

#include <linux/device/bus.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/multikernel.h>
#include <linux/overflow.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "internal.h"

struct mk_pci_assignment {
	struct list_head instance_node;
	struct list_head active_node;
	struct list_head transaction_node;
	struct mk_instance *instance;
	struct mk_pci_device *inventory;
	struct pci_dev *vf;
	struct pci_dev *pf;
	const struct device_driver *host_driver;
	struct iommu_group *iommu_group;
	struct iommu_domain *iommu_domain;
	char *host_driver_override;
	struct mutex iommu_mutex; /* Serializes IOMMU activation and teardown. */
	unsigned int iommu_mapped_regions;
	bool iommu_dma_owner;
	bool iommu_attached;
	bool iommu_override_active;
	struct work_struct failure_work;
	atomic_t failure_pending;
	bool assigned;
	bool inventory_moved;
	bool expected_unbind;
};

static DEFINE_MUTEX(mk_pci_lease_mutex);
static DEFINE_SPINLOCK(mk_pci_active_lock);
static LIST_HEAD(mk_pci_active_assignments);
static bool mk_pci_notifier_registered;

static bool mk_pci_device_live(struct pci_dev *pdev)
{
	return device_is_registered(&pdev->dev) &&
	       !pci_dev_is_disconnected(pdev) &&
	       pci_device_is_present(pdev);
}

static bool mk_pci_device_matches_bdf(const struct mk_pci_device *device,
				      u16 domain, u8 bus, u8 devfn)
{
	return device->domain == domain && device->bus == bus &&
	       device->slot == PCI_SLOT(devfn) &&
	       device->func == PCI_FUNC(devfn);
}

static struct mk_pci_device *
mk_pci_find_device_bdf(struct list_head *devices, u16 domain, u8 bus, u8 devfn)
{
	struct mk_pci_device *device;

	list_for_each_entry(device, devices, list) {
		if (mk_pci_device_matches_bdf(device, domain, bus, devfn))
			return device;
	}

	return NULL;
}

static bool mk_pci_inventory_matches(const struct mk_pci_device *left,
				     const struct mk_pci_device *right)
{
	return left->vendor == right->vendor &&
	       left->device == right->device &&
	       mk_pci_device_matches_bdf(left, right->domain, right->bus,
					 PCI_DEVFN(right->slot, right->func));
}

static struct mk_pci_device *
mk_pci_find_root_inventory(const struct mk_pci_device *requested)
{
	struct mk_pci_device *device;

	list_for_each_entry(device, &root_instance->pci_devices, list) {
		if (mk_pci_inventory_matches(device, requested))
			return device;
	}

	return NULL;
}

static struct mk_pci_device *
mk_pci_find_root_bdf(u16 domain, u8 bus, u8 devfn)
{
	return mk_pci_find_device_bdf(&root_instance->pci_devices, domain, bus,
				      devfn);
}

static struct mk_pci_assignment *
mk_pci_find_assignment(struct mk_instance *instance, u16 domain, u8 bus,
		       u8 devfn)
{
	struct mk_pci_assignment *assignment;

	list_for_each_entry(assignment, &instance->pci_assignments,
			    instance_node) {
		if (mk_pci_device_matches_bdf(assignment->inventory, domain, bus,
					      devfn))
			return assignment;
	}

	return NULL;
}

static void mk_pci_assignment_failure_work(struct work_struct *work)
{
	struct mk_pci_assignment *assignment =
		container_of(work, struct mk_pci_assignment, failure_work);
	struct mk_instance *instance = assignment->instance;
	int ret;

	pr_err("PCI assignment lease for %s was lost by instance %d (%s)\n",
	       pci_name(assignment->vf), instance->id, instance->name);

	if (READ_ONCE(instance->state) == MK_STATE_ACTIVE) {
		ret = mk_instance_force_halt(instance);
		if (ret)
			pr_err("Failed to force halt instance %d after PCI lease loss: %d\n",
			       instance->id, ret);
	}

	mk_instance_set_state(instance, MK_STATE_FAILED);
}

static void mk_pci_schedule_failure(struct mk_pci_assignment *assignment)
{
	if (atomic_cmpxchg(&assignment->failure_pending, 0, 1))
		return;

	if (!schedule_work(&assignment->failure_work))
		atomic_set(&assignment->failure_pending, 0);
}

static int mk_pci_bus_notify(struct notifier_block *nb, unsigned long action,
			     void *data)
{
	struct pci_dev *pdev = to_pci_dev(data);
	struct mk_pci_assignment *assignment;
	unsigned long flags;

	if (action != BUS_NOTIFY_DEL_DEVICE &&
	    action != BUS_NOTIFY_REMOVED_DEVICE &&
	    action != BUS_NOTIFY_UNBOUND_DRIVER)
		return NOTIFY_DONE;

	spin_lock_irqsave(&mk_pci_active_lock, flags);
	list_for_each_entry(assignment, &mk_pci_active_assignments,
			    active_node) {
		if (pdev != assignment->vf && pdev != assignment->pf)
			continue;
		if (pdev == assignment->vf &&
		    action == BUS_NOTIFY_UNBOUND_DRIVER &&
		    assignment->expected_unbind)
			continue;
		mk_pci_schedule_failure(assignment);
	}
	spin_unlock_irqrestore(&mk_pci_active_lock, flags);

	return NOTIFY_OK;
}

static struct notifier_block mk_pci_bus_notifier = {
	.notifier_call = mk_pci_bus_notify,
};

#if IS_ENABLED(CONFIG_IOMMU_API)
#define MK_PCI_ASSIGNMENT_DRIVER_NAME "multikernel-pci-assignment"

static int mk_pci_iommu_assignment_probe(struct pci_dev *pdev,
					 const struct pci_device_id *id);
static void mk_pci_iommu_assignment_remove(struct pci_dev *pdev);

static struct pci_driver mk_pci_assignment_driver = {
	.name = MK_PCI_ASSIGNMENT_DRIVER_NAME,
	.probe = mk_pci_iommu_assignment_probe,
	.remove = mk_pci_iommu_assignment_remove,
	.driver_managed_dma = true,
};

struct mk_pci_iommu_group_check {
	struct device *vf;
	unsigned int count;
};

static int mk_pci_iommu_check_group_device(struct device *dev, void *data)
{
	struct mk_pci_iommu_group_check *check = data;

	check->count++;
	return dev == check->vf ? 0 : -EXDEV;
}

static void mk_pci_iommu_free_resv_regions(struct list_head *regions)
{
	struct iommu_resv_region *region, *tmp;

	list_for_each_entry_safe(region, tmp, regions, list) {
		list_del(&region->list);
		kfree(region);
	}
}

static int mk_pci_iommu_validate_group(struct mk_pci_assignment *assignment)
{
	struct mk_pci_iommu_group_check check = {
		.vf = &assignment->vf->dev,
	};
	struct iommu_resv_region *region;
	LIST_HEAD(resv_regions);
	int ret;

	ret = iommu_group_for_each_dev(assignment->iommu_group, &check,
				       mk_pci_iommu_check_group_device);
	if (ret || check.count != 1) {
		pr_err("IOMMU group %d for %s is not an isolated singleton group\n",
		       iommu_group_id(assignment->iommu_group),
		       pci_name(assignment->vf));
		return ret ?: -EXDEV;
	}

	if (!iommu_group_has_isolated_msi(assignment->iommu_group)) {
		pr_err("IOMMU group %d for %s lacks isolated MSI delivery\n",
		       iommu_group_id(assignment->iommu_group),
		       pci_name(assignment->vf));
		return -EPERM;
	}

	ret = iommu_get_group_resv_regions(assignment->iommu_group,
					   &resv_regions);
	if (ret) {
		mk_pci_iommu_free_resv_regions(&resv_regions);
		return ret;
	}

	list_for_each_entry(region, &resv_regions, list) {
		if (region->type != IOMMU_RESV_DIRECT &&
		    region->type != IOMMU_RESV_DIRECT_RELAXABLE &&
		    region->type != IOMMU_RESV_SW_MSI)
			continue;
		pr_err("IOMMU group %d for %s requires unsupported reserved region %#llx-%#llx type %u\n",
		       iommu_group_id(assignment->iommu_group),
		       pci_name(assignment->vf),
		       (unsigned long long)region->start,
		       (unsigned long long)(region->start + region->length - 1),
		       region->type);
		ret = -EPERM;
		break;
	}

	mk_pci_iommu_free_resv_regions(&resv_regions);
	return ret;
}

static int
mk_pci_iommu_validate_region(struct mk_pci_assignment *assignment,
			     const struct mk_memory_region *region)
{
	struct iommu_domain *domain = assignment->iommu_domain;
	resource_size_t start = region->res.start;
	resource_size_t size = resource_size(&region->res);
	u64 dma_mask = dma_get_mask(&assignment->vf->dev);
	u64 end;
	unsigned long min_page_size;

	if (!size || check_add_overflow((u64)start, (u64)size - 1, &end))
		return -EOVERFLOW;
	if (!domain->pgsize_bitmap)
		return -EOPNOTSUPP;

	min_page_size = 1UL << __ffs(domain->pgsize_bitmap);
	if (!IS_ALIGNED(start, min_page_size) ||
	    !IS_ALIGNED(size, min_page_size)) {
		pr_err("Instance %d memory %#llx-%#llx is not aligned to IOMMU page size %#lx\n",
		       assignment->instance->id, (unsigned long long)start,
		       (unsigned long long)end, min_page_size);
		return -EINVAL;
	}
	if (start > ULONG_MAX || end > ULONG_MAX || end > dma_mask) {
		pr_err("Instance %d memory %#llx-%#llx exceeds DMA addressability of %s\n",
		       assignment->instance->id, (unsigned long long)start,
		       (unsigned long long)end, pci_name(assignment->vf));
		return -ERANGE;
	}
	if (domain->geometry.force_aperture &&
	    (start < domain->geometry.aperture_start ||
	     end > domain->geometry.aperture_end)) {
		pr_err("Instance %d memory %#llx-%#llx is outside the IOMMU aperture for %s\n",
		       assignment->instance->id, (unsigned long long)start,
		       (unsigned long long)end, pci_name(assignment->vf));
		return -ERANGE;
	}

	return 0;
}

static void mk_pci_iommu_unmap_regions(struct mk_pci_assignment *assignment)
{
	struct mk_memory_region *region;
	unsigned int remaining = assignment->iommu_mapped_regions;

	list_for_each_entry(region, &assignment->instance->memory_regions, list) {
		resource_size_t size;
		size_t unmapped;

		if (!remaining)
			break;
		size = resource_size(&region->res);
		unmapped = iommu_unmap(assignment->iommu_domain,
				       region->res.start, size);
		if (unmapped != size)
			pr_err("IOMMU unmapped only %#zx of %#llx bytes for instance %d at %#llx\n",
			       unmapped, (unsigned long long)size,
			       assignment->instance->id,
			       (unsigned long long)region->res.start);
		remaining--;
	}
	if (remaining)
		pr_err("IOMMU lease for %s lost %u mapped instance regions\n",
		       pci_name(assignment->vf), remaining);
	assignment->iommu_mapped_regions = 0;
}

static void
__mk_pci_iommu_deactivate_assignment(struct mk_pci_assignment *assignment)
{
	if (assignment->iommu_attached) {
		iommu_detach_group(assignment->iommu_domain,
				   assignment->iommu_group);
		assignment->iommu_attached = false;
	}
	if (assignment->iommu_dma_owner) {
		iommu_device_release_dma_owner(&assignment->vf->dev);
		assignment->iommu_dma_owner = false;
	}
}

static void
mk_pci_iommu_deactivate_assignment(struct mk_pci_assignment *assignment)
{
	mutex_lock(&assignment->iommu_mutex);
	__mk_pci_iommu_deactivate_assignment(assignment);
	mutex_unlock(&assignment->iommu_mutex);
}

static void mk_pci_iommu_release_assignment(struct mk_pci_assignment *assignment)
{
	mutex_lock(&assignment->iommu_mutex);
	__mk_pci_iommu_deactivate_assignment(assignment);

	if (assignment->iommu_domain) {
		mk_pci_iommu_unmap_regions(assignment);
		iommu_domain_free(assignment->iommu_domain);
		assignment->iommu_domain = NULL;
	}
	if (assignment->iommu_group) {
		iommu_group_put(assignment->iommu_group);
		assignment->iommu_group = NULL;
	}
	mutex_unlock(&assignment->iommu_mutex);
}

static int mk_pci_iommu_fault(struct iommu_domain *domain,
			      struct device *dev, unsigned long iova,
			      int flags, void *token)
{
	struct mk_pci_assignment *assignment = token;

	if (WARN_ON_ONCE(domain != assignment->iommu_domain))
		return -EINVAL;
	pr_err_ratelimited("IOMMU fault from %s at IOVA %#lx (%s) for instance %d\n",
			   dev_name(dev), iova,
			   flags & IOMMU_FAULT_WRITE ? "write" : "read",
			   assignment->instance->id);
	mk_pci_schedule_failure(assignment);
	return 0;
}

static int mk_pci_iommu_prepare_assignment(struct mk_pci_assignment *assignment)
{
	struct mk_memory_region *region;
	int ret;

	if (!device_iommu_mapped(&assignment->vf->dev)) {
		pr_err("Cannot assign %s without an active hardware IOMMU\n",
		       pci_name(assignment->vf));
		return -EOPNOTSUPP;
	}
	if (!assignment->instance->region_count ||
	    list_empty(&assignment->instance->memory_regions))
		return -EINVAL;

	assignment->iommu_group = iommu_group_get(&assignment->vf->dev);
	if (!assignment->iommu_group)
		return -ENODEV;

	ret = mk_pci_iommu_validate_group(assignment);
	if (ret)
		goto err_release;

	assignment->iommu_domain =
		iommu_paging_domain_alloc(&assignment->vf->dev);
	if (IS_ERR(assignment->iommu_domain)) {
		ret = PTR_ERR(assignment->iommu_domain);
		assignment->iommu_domain = NULL;
		goto err_release;
	}

	list_for_each_entry(region, &assignment->instance->memory_regions, list) {
		resource_size_t size = resource_size(&region->res);

		ret = mk_pci_iommu_validate_region(assignment, region);
		if (ret)
			goto err_release;
		ret = iommu_map(assignment->iommu_domain, region->res.start,
				region->res.start, size, IOMMU_READ | IOMMU_WRITE,
				GFP_KERNEL);
		if (ret)
			goto err_release;
		assignment->iommu_mapped_regions++;
	}

	iommu_set_fault_handler(assignment->iommu_domain,
				mk_pci_iommu_fault, assignment);
	pr_info("Prepared host IOMMU domain for %s with %u instance regions\n",
		pci_name(assignment->vf), assignment->iommu_mapped_regions);
	return 0;

err_release:
	mk_pci_iommu_release_assignment(assignment);
	return ret;
}

static int mk_pci_iommu_commit_assignment(struct mk_pci_assignment *assignment)
{
	int ret;

	if (!assignment->iommu_domain)
		return 0;

	if (assignment->vf->driver_override) {
		assignment->host_driver_override =
			kstrdup(assignment->vf->driver_override, GFP_KERNEL);
		if (!assignment->host_driver_override)
			return -ENOMEM;
	}

	ret = driver_set_override(&assignment->vf->dev,
				  &assignment->vf->driver_override,
				  MK_PCI_ASSIGNMENT_DRIVER_NAME,
				  strlen(MK_PCI_ASSIGNMENT_DRIVER_NAME));
	if (ret)
		return ret;
	assignment->iommu_override_active = true;
	pci_set_drvdata(assignment->vf, assignment);
	ret = device_driver_attach(&mk_pci_assignment_driver.driver,
				   &assignment->vf->dev);
	if (ret)
		return ret;
	if (assignment->vf->dev.driver != &mk_pci_assignment_driver.driver)
		return -ENODEV;
	return 0;
}

static int mk_pci_iommu_assignment_probe(struct pci_dev *pdev,
					 const struct pci_device_id *id)
{
	struct mk_pci_assignment *assignment = pci_get_drvdata(pdev);
	int ret;

	if (!assignment || assignment->vf != pdev || !assignment->iommu_domain)
		return -ENODEV;
	ret = iommu_device_claim_dma_owner(&assignment->vf->dev, assignment);
	if (ret)
		return ret;
	assignment->iommu_dma_owner = true;

	ret = iommu_attach_group(assignment->iommu_domain,
				 assignment->iommu_group);
	if (ret)
		return ret;
	assignment->iommu_attached = true;
	pr_info("Attached %s to host-owned IOMMU domain for instance %d\n",
		pci_name(assignment->vf), assignment->instance->id);
	return 0;
}

static void mk_pci_iommu_assignment_remove(struct pci_dev *pdev)
{
	struct mk_pci_assignment *assignment = pci_get_drvdata(pdev);

	if (assignment && assignment->vf == pdev) {
		mk_pci_iommu_deactivate_assignment(assignment);
		pci_set_drvdata(pdev, NULL);
	}
}

static int
mk_pci_iommu_restore_host_binding(struct mk_pci_assignment *assignment)
{
	struct pci_dev *vf = assignment->vf;
	const char *override = assignment->host_driver_override ?: "";
	unsigned long flags;
	int ret = 0;

	if (vf->dev.driver == &mk_pci_assignment_driver.driver) {
		spin_lock_irqsave(&mk_pci_active_lock, flags);
		assignment->expected_unbind = true;
		spin_unlock_irqrestore(&mk_pci_active_lock, flags);
		device_release_driver(&vf->dev);
		spin_lock_irqsave(&mk_pci_active_lock, flags);
		assignment->expected_unbind = false;
		spin_unlock_irqrestore(&mk_pci_active_lock, flags);
	} else if (vf->dev.driver) {
		pr_err("Cannot release assignment driver from %s: device is bound to %s\n",
		       pci_name(vf), vf->dev.driver->name);
		return -EBUSY;
	}

	pci_set_drvdata(vf, NULL);
	if (assignment->iommu_override_active) {
		ret = driver_set_override(&vf->dev, &vf->driver_override,
					  override, strlen(override));
		if (ret)
			return ret;
		assignment->iommu_override_active = false;
	}
	return 0;
}

static int mk_pci_iommu_system_init(void)
{
	return pci_register_driver(&mk_pci_assignment_driver);
}

static void mk_pci_iommu_system_cleanup(void)
{
	pci_unregister_driver(&mk_pci_assignment_driver);
}
#else
static int mk_pci_iommu_prepare_assignment(struct mk_pci_assignment *assignment)
{
	pr_err("Cannot assign %s without CONFIG_IOMMU_API\n",
	       pci_name(assignment->vf));
	return -EOPNOTSUPP;
}

static int mk_pci_iommu_commit_assignment(struct mk_pci_assignment *assignment)
{
	return 0;
}

static void mk_pci_iommu_release_assignment(struct mk_pci_assignment *assignment)
{
}

static int
mk_pci_iommu_restore_host_binding(struct mk_pci_assignment *assignment)
{
	return 0;
}

static int mk_pci_iommu_system_init(void)
{
	return 0;
}

static void mk_pci_iommu_system_cleanup(void)
{
}
#endif

static int
mk_pci_prepare_assignment(struct mk_instance *instance,
			  const struct mk_pci_device *requested,
			  struct list_head *transaction)
{
	struct mk_pci_assignment *assignment;
	struct mk_pci_device *inventory;
	struct pci_dev *vf;
	struct pci_dev *pf;
	struct pci_dev *physfn;
	int ret;

	inventory = mk_pci_find_root_inventory(requested);
	if (!inventory) {
		pr_err("PCI device %04x:%04x@%04x:%02x:%02x.%x is not available in the root pool\n",
		       requested->vendor, requested->device, requested->domain,
		       requested->bus, requested->slot, requested->func);
		return -ENOENT;
	}

	vf = pci_get_domain_bus_and_slot(inventory->domain,
					 inventory->bus,
					 PCI_DEVFN(inventory->slot,
						   inventory->func));
	if (!vf)
		return -ENODEV;

	if (vf->vendor != inventory->vendor ||
	    vf->device != inventory->device) {
		pr_err("PCI identity changed for %s: expected %04x:%04x, found %04x:%04x\n",
		       pci_name(vf), inventory->vendor, inventory->device,
		       vf->vendor, vf->device);
		pci_dev_put(vf);
		return -ENODEV;
	}

	physfn = pci_physfn(vf);
	if (!vf->is_virtfn || physfn == vf) {
		pr_err("PCI assignment only supports SR-IOV VFs, rejecting %s\n",
		       pci_name(vf));
		pci_dev_put(vf);
		return -EOPNOTSUPP;
	}

	if (!mk_pci_device_live(vf) || !mk_pci_device_live(physfn)) {
		pci_dev_put(vf);
		return -ENODEV;
	}

	if (pci_is_dev_assigned(vf) ||
	    mk_pci_find_assignment(instance, inventory->domain,
				   inventory->bus,
				   PCI_DEVFN(inventory->slot,
					     inventory->func))) {
		pci_dev_put(vf);
		return -EBUSY;
	}

	pf = pci_dev_get(physfn);
	assignment = kzalloc(sizeof(*assignment), GFP_KERNEL);
	if (!assignment) {
		pci_dev_put(pf);
		pci_dev_put(vf);
		return -ENOMEM;
	}

	assignment->instance = instance;
	assignment->inventory = inventory;
	assignment->vf = vf;
	assignment->pf = pf;
	assignment->host_driver = vf->dev.driver;
	if (assignment->host_driver && assignment->host_driver->owner &&
	    !try_module_get(assignment->host_driver->owner)) {
		kfree(assignment);
		pci_dev_put(pf);
		pci_dev_put(vf);
		return -ENODEV;
	}

	INIT_LIST_HEAD(&assignment->instance_node);
	INIT_LIST_HEAD(&assignment->active_node);
	INIT_LIST_HEAD(&assignment->transaction_node);
	mutex_init(&assignment->iommu_mutex);
	INIT_WORK(&assignment->failure_work, mk_pci_assignment_failure_work);
	atomic_set(&assignment->failure_pending, 0);

	ret = mk_pci_iommu_prepare_assignment(assignment);
	if (ret)
		goto err_module;

	list_add_tail(&assignment->instance_node, &instance->pci_assignments);
	list_add_tail(&assignment->transaction_node, transaction);

	return 0;

err_module:
	if (assignment->host_driver && assignment->host_driver->owner)
		module_put(assignment->host_driver->owner);
	kfree(assignment);
	pci_dev_put(pf);
	pci_dev_put(vf);
	return ret;
}

static int mk_pci_commit_assignment(struct mk_pci_assignment *assignment)
{
	struct pci_dev *vf = assignment->vf;
	unsigned long flags;
	int ret;
	int i;

	if (!mk_pci_device_live(vf) || !mk_pci_device_live(assignment->pf))
		return -ENODEV;

	if (vf->dev.driver != assignment->host_driver ||
	    pci_is_dev_assigned(vf))
		return -EBUSY;

	pci_set_dev_assigned(vf);
	assignment->assigned = true;

	spin_lock_irqsave(&mk_pci_active_lock, flags);
	list_add_tail(&assignment->active_node, &mk_pci_active_assignments);
	assignment->expected_unbind = true;
	spin_unlock_irqrestore(&mk_pci_active_lock, flags);

	if (assignment->host_driver)
		device_release_driver(&vf->dev);

	spin_lock_irqsave(&mk_pci_active_lock, flags);
	assignment->expected_unbind = false;
	spin_unlock_irqrestore(&mk_pci_active_lock, flags);

	if (vf->dev.driver)
		return -EBUSY;

	ret = mk_pci_iommu_commit_assignment(assignment);
	if (ret)
		return ret;

	for (i = 0; i < MK_PCI_RESOURCE_COUNT; i++) {
		assignment->inventory->resources[i].start =
			vf->resource[i].start;
		assignment->inventory->resources[i].end =
			vf->resource[i].end;
		assignment->inventory->resources[i].flags =
			vf->resource[i].flags;
	}
	assignment->inventory->resources_valid = true;

	list_move_tail(&assignment->inventory->list,
		       &assignment->instance->pci_devices);
	root_instance->pci_device_count--;
	assignment->instance->pci_device_count++;
	assignment->instance->pci_devices_valid = true;
	assignment->inventory_moved = true;

	pr_info("Leased SR-IOV VF %s to instance %d (%s)\n",
		pci_name(vf), assignment->instance->id,
		assignment->instance->name);
	return 0;
}

static int mk_pci_release_assignment(struct mk_pci_assignment *assignment)
{
	struct mk_instance *instance = assignment->instance;
	struct pci_dev *vf = assignment->vf;
	unsigned long flags;
	int binding_ret;
	int ret = 0;

	spin_lock_irqsave(&mk_pci_active_lock, flags);
	if (!list_empty(&assignment->active_node))
		list_del_init(&assignment->active_node);
	assignment->expected_unbind = false;
	spin_unlock_irqrestore(&mk_pci_active_lock, flags);

	if (assignment->assigned) {
		pci_clear_dev_assigned(vf);
		assignment->assigned = false;
	}

	mk_pci_iommu_release_assignment(assignment);
	cancel_work_sync(&assignment->failure_work);
	binding_ret = mk_pci_iommu_restore_host_binding(assignment);
	if (binding_ret)
		ret = binding_ret;

	if (!binding_ret && assignment->host_driver && mk_pci_device_live(vf)) {
		if (!vf->dev.driver) {
			ret = device_driver_attach(assignment->host_driver,
						   &vf->dev);
			if (ret)
				pr_err("Failed to restore driver %s to %s: %d\n",
				       assignment->host_driver->name,
				       pci_name(vf), ret);
		} else if (vf->dev.driver != assignment->host_driver) {
			pr_err("Cannot restore driver %s to %s: device is bound to %s\n",
			       assignment->host_driver->name, pci_name(vf),
			       vf->dev.driver->name);
			ret = -EBUSY;
		}
	}

	if (assignment->inventory_moved && root_instance) {
		list_move_tail(&assignment->inventory->list,
			       &root_instance->pci_devices);
		instance->pci_device_count--;
		root_instance->pci_device_count++;
		root_instance->pci_devices_valid = true;
		assignment->inventory_moved = false;
	}

	kfree(assignment->host_driver_override);
	if (assignment->host_driver && assignment->host_driver->owner)
		module_put(assignment->host_driver->owner);
	if (!list_empty(&assignment->transaction_node))
		list_del_init(&assignment->transaction_node);
	list_del_init(&assignment->instance_node);
	pci_dev_put(assignment->pf);
	pci_dev_put(vf);
	kfree(assignment);

	return ret;
}

static void mk_pci_rollback_transaction(struct list_head *transaction)
{
	struct mk_pci_assignment *assignment;

	while (!list_empty(transaction)) {
		assignment = list_last_entry(transaction,
					     struct mk_pci_assignment,
					     transaction_node);
		mk_pci_release_assignment(assignment);
	}
}

static int mk_pci_commit_transaction(struct list_head *transaction)
{
	struct mk_pci_assignment *assignment;
	int ret;

	list_for_each_entry(assignment, transaction, transaction_node) {
		ret = mk_pci_commit_assignment(assignment);
		if (ret)
			return ret;
	}

	while (!list_empty(transaction)) {
		assignment = list_first_entry(transaction,
					      struct mk_pci_assignment,
					      transaction_node);
		list_del_init(&assignment->transaction_node);
	}

	return 0;
}

void mk_pci_lease_instance_init(struct mk_instance *instance)
{
	mutex_init(&instance->resource_mutex);
	INIT_LIST_HEAD(&instance->pci_assignments);
}

bool mk_pci_iommu_lease_active_locked(struct mk_instance *instance)
{
	if (!instance)
		return false;

	lockdep_assert_held(&instance->resource_mutex);
	return !list_empty(&instance->pci_assignments);
}

int mk_pci_assign_devices(struct mk_instance *instance,
			  const struct list_head *requested_devices,
			  int requested_count)
{
	struct mk_pci_device *requested;
	LIST_HEAD(transaction);
	int prepared = 0;
	int ret = 0;

	if (!instance || instance == root_instance || !requested_devices ||
	    requested_count < 0)
		return -EINVAL;
	if (!root_instance || !root_instance->pci_devices_valid)
		return -EINVAL;

	mutex_lock(&instance->resource_mutex);
	mutex_lock(&mk_pci_lease_mutex);
	pci_lock_rescan_remove();

	list_for_each_entry(requested, requested_devices, list) {
		ret = mk_pci_prepare_assignment(instance, requested,
						&transaction);
		if (ret)
			goto rollback;
		prepared++;
	}

	if (prepared != requested_count) {
		ret = -EINVAL;
		goto rollback;
	}

	ret = mk_pci_commit_transaction(&transaction);
	if (ret)
		goto rollback;
	goto out;

rollback:
	mk_pci_rollback_transaction(&transaction);
out:
	pci_unlock_rescan_remove();
	mutex_unlock(&mk_pci_lease_mutex);
	mutex_unlock(&instance->resource_mutex);
	return ret;
}

int mk_pci_assign_device(struct mk_instance *instance, u16 domain, u8 bus,
			 u8 devfn)
{
	struct mk_pci_device *inventory;
	LIST_HEAD(transaction);
	int ret;

	if (!instance || instance == root_instance)
		return -EINVAL;

	mutex_lock(&instance->resource_mutex);
	mutex_lock(&mk_pci_lease_mutex);
	pci_lock_rescan_remove();

	if (instance->state != MK_STATE_READY) {
		ret = -EBUSY;
		goto out;
	}
	if (!root_instance || !root_instance->pci_devices_valid) {
		ret = -EINVAL;
		goto out;
	}

	inventory = mk_pci_find_root_bdf(domain, bus, devfn);
	if (!inventory) {
		ret = -ENOENT;
		goto out;
	}

	ret = mk_pci_prepare_assignment(instance, inventory, &transaction);
	if (ret)
		goto rollback;
	ret = mk_pci_commit_transaction(&transaction);
	if (ret)
		goto rollback;
	goto out;

rollback:
	mk_pci_rollback_transaction(&transaction);
out:
	pci_unlock_rescan_remove();
	mutex_unlock(&mk_pci_lease_mutex);
	mutex_unlock(&instance->resource_mutex);
	return ret;
}

int mk_pci_unassign_device(struct mk_instance *instance, u16 domain, u8 bus,
			   u8 devfn)
{
	struct mk_pci_assignment *assignment;
	int ret;

	if (!instance || instance == root_instance)
		return -EINVAL;

	mutex_lock(&instance->resource_mutex);
	mutex_lock(&mk_pci_lease_mutex);
	pci_lock_rescan_remove();

	if (instance->state != MK_STATE_READY) {
		ret = -EBUSY;
		goto out;
	}

	assignment = mk_pci_find_assignment(instance, domain, bus, devfn);
	if (!assignment) {
		ret = -ENOENT;
		goto out;
	}

	ret = mk_pci_release_assignment(assignment);
out:
	pci_unlock_rescan_remove();
	mutex_unlock(&mk_pci_lease_mutex);
	mutex_unlock(&instance->resource_mutex);
	return ret;
}

void mk_pci_release_assignments(struct mk_instance *instance)
{
	struct mk_pci_assignment *assignment;

	if (!instance || instance == root_instance)
		return;

	mutex_lock(&instance->resource_mutex);
	mutex_lock(&mk_pci_lease_mutex);
	pci_lock_rescan_remove();
	while (!list_empty(&instance->pci_assignments)) {
		assignment = list_last_entry(&instance->pci_assignments,
					     struct mk_pci_assignment,
					     instance_node);
		mk_pci_release_assignment(assignment);
	}
	pci_unlock_rescan_remove();
	mutex_unlock(&mk_pci_lease_mutex);
	mutex_unlock(&instance->resource_mutex);
}

int mk_pci_lease_system_init(void)
{
	int ret;

	ret = mk_pci_iommu_system_init();
	if (ret)
		return ret;
	ret = bus_register_notifier(&pci_bus_type, &mk_pci_bus_notifier);
	if (ret) {
		mk_pci_iommu_system_cleanup();
		return ret;
	}
	mk_pci_notifier_registered = true;
	return 0;
}

void mk_pci_lease_system_cleanup(void)
{
	if (mk_pci_notifier_registered) {
		bus_unregister_notifier(&pci_bus_type, &mk_pci_bus_notifier);
		mk_pci_notifier_registered = false;
	}
	mk_pci_iommu_system_cleanup();
}

static struct mk_pci_device *mk_pci_find_assigned(struct pci_bus *bus, int devfn)
{
	if (!root_instance || !root_instance->dtb_data ||
	    !root_instance->pci_devices_valid)
		return NULL;

	return mk_pci_find_device_bdf(&root_instance->pci_devices,
				      pci_domain_nr(bus), bus->number, devfn);
}

/**
 * mk_pci_get_assigned_identity - Get the identity presented to an instance
 * @bus: PCI bus
 * @devfn: device/function number
 * @vendor: assigned Vendor ID
 * @device_id: assigned Device ID
 *
 * Returns: true when assignment metadata contains an exact location match.
 */
bool mk_pci_get_assigned_identity(struct pci_bus *bus, int devfn,
				  u16 *vendor, u16 *device_id)
{
	struct mk_pci_device *device = mk_pci_find_assigned(bus, devfn);

	if (!device)
		return false;

	if (vendor)
		*vendor = device->vendor;
	if (device_id)
		*device_id = device->device;
	return true;
}

static void mk_pci_restore_resources(struct pci_dev *dev)
{
	struct mk_pci_device *device;
	int i;

	device = mk_pci_find_assigned(dev->bus, dev->devfn);
	if (!device || !device->resources_valid)
		return;

	dev->non_compliant_bars = true;
	for (i = 0; i < MK_PCI_RESOURCE_COUNT; i++) {
		dev->resource[i].start = device->resources[i].start;
		dev->resource[i].end = device->resources[i].end;
		dev->resource[i].flags = device->resources[i].flags;
	}
	pr_info("Restored PCI BAR resources for %s\n", pci_name(dev));
}

DECLARE_PCI_FIXUP_EARLY(PCI_ANY_ID, PCI_ANY_ID, mk_pci_restore_resources);
