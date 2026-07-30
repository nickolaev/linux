// SPDX-License-Identifier: GPL-2.0-only
/*
 * Multikernel PCI assignment policy
 *
 * Keeps assigned-device discovery, identity presentation, bridge traversal,
 * and resource restoration independent from the manifest that populated
 * the current instance.
 */

#include <linux/device/bus.h>
#include <linux/module.h>
#include <linux/multikernel.h>
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
	INIT_WORK(&assignment->failure_work, mk_pci_assignment_failure_work);
	atomic_set(&assignment->failure_pending, 0);
	list_add_tail(&assignment->instance_node, &instance->pci_assignments);
	list_add_tail(&assignment->transaction_node, transaction);

	return 0;
}

static int mk_pci_commit_assignment(struct mk_pci_assignment *assignment)
{
	struct pci_dev *vf = assignment->vf;
	unsigned long flags;
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
	int ret = 0;

	spin_lock_irqsave(&mk_pci_active_lock, flags);
	if (!list_empty(&assignment->active_node))
		list_del_init(&assignment->active_node);
	assignment->expected_unbind = false;
	spin_unlock_irqrestore(&mk_pci_active_lock, flags);

	cancel_work_sync(&assignment->failure_work);

	if (assignment->assigned) {
		pci_clear_dev_assigned(vf);
		assignment->assigned = false;
	}

	if (assignment->host_driver && mk_pci_device_live(vf)) {
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
	INIT_LIST_HEAD(&instance->pci_assignments);
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
	return ret;
}

int mk_pci_unassign_device(struct mk_instance *instance, u16 domain, u8 bus,
			   u8 devfn)
{
	struct mk_pci_assignment *assignment;
	int ret;

	if (!instance || instance == root_instance)
		return -EINVAL;

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
	return ret;
}

void mk_pci_release_assignments(struct mk_instance *instance)
{
	struct mk_pci_assignment *assignment;

	if (!instance || instance == root_instance)
		return;

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
}

int mk_pci_lease_system_init(void)
{
	int ret;

	ret = bus_register_notifier(&pci_bus_type, &mk_pci_bus_notifier);
	if (!ret)
		mk_pci_notifier_registered = true;
	return ret;
}

void mk_pci_lease_system_cleanup(void)
{
	if (!mk_pci_notifier_registered)
		return;
	bus_unregister_notifier(&pci_bus_type, &mk_pci_bus_notifier);
	mk_pci_notifier_registered = false;
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

	*vendor = device->vendor;
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

static bool mk_pci_bridge_reaches_assigned(struct pci_bus *bus, int devfn,
					   const struct pci_ops *ops)
{
	struct mk_pci_device *device;
	u16 domain = pci_domain_nr(bus);
	u8 secondary_bus = 0;
	u8 subordinate_bus = 0;
	u8 hdr_type;
	u32 value;

	if (ops->read(bus, devfn, PCI_HEADER_TYPE, sizeof(hdr_type), &value))
		return false;
	hdr_type = value;
	if ((hdr_type & PCI_HEADER_TYPE_MASK) != PCI_HEADER_TYPE_BRIDGE)
		return false;

	if (ops->read(bus, devfn, PCI_SECONDARY_BUS, sizeof(secondary_bus),
		      &value))
		return false;
	secondary_bus = value;
	if (ops->read(bus, devfn, PCI_SUBORDINATE_BUS,
		      sizeof(subordinate_bus), &value))
		return false;
	subordinate_bus = value;
	if (!secondary_bus || subordinate_bus < secondary_bus)
		return false;

	list_for_each_entry(device, &root_instance->pci_devices, list) {
		if (device->domain == domain && device->bus >= secondary_bus &&
		    device->bus <= subordinate_bus)
			return true;
	}

	return false;
}

/**
 * mk_pci_should_probe - Check whether PCI probing may access a location
 * @bus: PCI bus
 * @devfn: device/function number
 * @ops: unfiltered config-space operations used to identify bridge paths
 *
 * Exact assigned functions and bridges leading to downstream assignments are
 * visible. Other functions are rejected before their config space is read.
 *
 * Returns: true if probing should proceed, false to skip entirely.
 */
bool mk_pci_should_probe(struct pci_bus *bus, int devfn,
			 const struct pci_ops *ops)
{
	struct mk_pci_device *device;
	u16 domain = pci_domain_nr(bus);

	if (!ops || !root_instance || !root_instance->dtb_data)
		return true;
	if (!root_instance->pci_devices_valid ||
	    !root_instance->pci_device_count)
		return false;
	if (mk_pci_find_assigned(bus, devfn))
		return true;

	list_for_each_entry(device, &root_instance->pci_devices, list) {
		if (device->domain == domain && device->bus > bus->number)
			return mk_pci_bridge_reaches_assigned(bus, devfn, ops);
	}

	return false;
}
