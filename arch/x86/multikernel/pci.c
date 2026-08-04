// SPDX-License-Identifier: GPL-2.0-only
/*
 * x86 PCI support for multikernel spawn kernels.
 *
 * This file owns the x86-specific transport of PCI ECAM windows. Generic
 * MMCONFIG code only provides an iterator over its region list.
 */

#include <linux/init.h>
#include <linux/ktime.h>
#include <linux/multikernel.h>
#include <linux/panic.h>
#include <linux/pci.h>

#include <asm/multikernel.h>
#include <asm/pci.h>
#include <asm/pci_x86.h>
#include <asm/topology.h>
#include <asm/x86_init.h>

#ifdef CONFIG_PCI_MMCONFIG
struct mk_mmcfg_snapshot {
	const struct mk_instance *instance;
	struct mk_pci_host_bridge *bridges;
	size_t capacity;
	size_t count;
};

static bool mk_pci_roots_ready;
static atomic64_t mk_pci_request_id = ATOMIC64_INIT(0);
static atomic64_t mk_pci_cfg_count = ATOMIC64_INIT(0);
static atomic64_t mk_pci_cfg_total_ns = ATOMIC64_INIT(0);
static atomic64_t mk_pci_cfg_max_ns = ATOMIC64_INIT(0);
static LIST_HEAD(mk_pci_cfg_pending);
static DEFINE_RAW_SPINLOCK(mk_pci_cfg_pending_lock);
static struct mk_instance *mk_pci_host_instance;

struct mk_pci_cfg_pending {
	struct list_head node;
	u64 request_id;
	s32 status;
	u32 value;
	bool done;
};

static bool mk_pci_message_from_host(mk_phys_cpu_t sender_cpu)
{
	return mk_pci_host_instance &&
		mk_cpu_set_contains(mk_pci_host_instance->cpus, sender_cpu);
}

static void mk_pci_cfg_response_handler(u32 msg_type, u32 subtype,
					void *payload, u32 payload_len,
					mk_phys_cpu_t sender_cpu, void *ctx)
{
	struct mk_pci_cfg_response *response = payload;
	struct mk_pci_cfg_pending *pending;
	unsigned long flags;

	if (msg_type != MK_MSG_PCI || subtype != MK_PCI_CFG_RESPONSE ||
	    !mk_pci_message_from_host(sender_cpu) ||
	    payload_len != sizeof(*response))
		return;

	raw_spin_lock_irqsave(&mk_pci_cfg_pending_lock, flags);
	list_for_each_entry(pending, &mk_pci_cfg_pending, node) {
		if (pending->request_id != response->request_id)
			continue;
		pending->status = response->status;
		pending->value = response->value;
		/* Publish the response fields before waking the polling CPU. */
		smp_store_release(&pending->done, true);
		break;
	}
	raw_spin_unlock_irqrestore(&mk_pci_cfg_pending_lock, flags);
}

static void mk_pci_record_latency(u64 start)
{
	u64 elapsed = ktime_get_mono_fast_ns() - start;
	u64 old_max = atomic64_read(&mk_pci_cfg_max_ns);

	atomic64_inc(&mk_pci_cfg_count);
	atomic64_add(elapsed, &mk_pci_cfg_total_ns);
	while (elapsed > old_max) {
		u64 previous = atomic64_cmpxchg(&mk_pci_cfg_max_ns, old_max,
						 elapsed);

		if (previous == old_max)
			break;
		old_max = previous;
	}
}

static int mk_pci_remote_config(unsigned int domain, unsigned int bus,
				unsigned int devfn, int where, int size,
				bool write, u32 *value)
{
	struct mk_pci_cfg_request request = {
		.request_id = atomic64_inc_return(&mk_pci_request_id),
		.sender_instance_id = root_instance ? root_instance->id : -1,
		.domain = domain,
		.bus = bus,
		.devfn = devfn,
		.reg = where,
		.len = size,
		.write = write,
		.value = *value,
	};
	struct mk_pci_cfg_pending pending = {
		.request_id = request.request_id,
		.status = PCIBIOS_SET_FAILED,
		.value = ~0U,
	};
	unsigned long flags;
	u64 start = ktime_get_mono_fast_ns();
	u64 deadline = start + NSEC_PER_SEC;
	int ret;

	raw_spin_lock_irqsave(&mk_pci_cfg_pending_lock, flags);
	list_add_tail(&pending.node, &mk_pci_cfg_pending);
	raw_spin_unlock_irqrestore(&mk_pci_cfg_pending_lock, flags);

	ret = mk_send_message(0, MK_MSG_PCI, MK_PCI_CFG_REQUEST,
			      &request, sizeof(request));
	if (ret)
		goto out;

	/* Pairs with the response handler's publication of status and value. */
	while (!smp_load_acquire(&pending.done)) {
		mk_poll_ipi_messages();
		if (ktime_get_mono_fast_ns() >= deadline) {
			ret = -ETIMEDOUT;
			goto out;
		}
		cpu_relax();
	}
	ret = pending.status;
	if (!write)
		*value = pending.value;
	mk_pci_record_latency(start);
out:
	raw_spin_lock_irqsave(&mk_pci_cfg_pending_lock, flags);
	list_del(&pending.node);
	raw_spin_unlock_irqrestore(&mk_pci_cfg_pending_lock, flags);
	if (ret < 0) {
		pr_err_ratelimited("Multikernel PCI config request timed out or failed to send: %d\n",
				   ret);
		return PCIBIOS_SET_FAILED;
	}
	return ret;
}

static bool mk_mmcfg_snapshot_contains(const struct mk_mmcfg_snapshot *snapshot,
				       u16 segment, u8 bus)
{
	size_t index;

	for (index = 0; index < snapshot->count; index++) {
		if (snapshot->bridges[index].segment == segment &&
		    snapshot->bridges[index].bus_start == bus)
			return true;
	}

	return false;
}

static bool mk_pci_identity_read(u16 vendor, u16 device, int where, int size,
				 u32 *value)
{
	u32 identity;
	u32 mask;

	if (where < PCI_VENDOR_ID || where + size > PCI_COMMAND)
		return false;

	identity = vendor | (u32)device << 16;
	mask = size == sizeof(identity) ? ~0U : (1U << (size * 8)) - 1;
	*value = (identity >> (where * 8)) & mask;
	return true;
}

static int mk_pci_raw_read(unsigned int domain, unsigned int bus,
			   unsigned int devfn, int where, int size,
			   u32 *value)
{
	u16 vendor, device;

	if (!mk_pci_get_assigned_identity_bdf(domain, bus, devfn, &vendor,
					      &device)) {
		*value = ~0U;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	if (mk_pci_identity_read(vendor, device, where, size, value))
		return PCIBIOS_SUCCESSFUL;

	return mk_pci_remote_config(domain, bus, devfn, where, size, false,
				    value);
}

static int mk_pci_raw_write(unsigned int domain, unsigned int bus,
			    unsigned int devfn, int where, int size,
			    u32 value)
{
	if (!mk_pci_get_assigned_identity_bdf(domain, bus, devfn, NULL, NULL))
		return PCIBIOS_DEVICE_NOT_FOUND;

	return mk_pci_remote_config(domain, bus, devfn, where, size, true,
				    &value);
}

static int mk_pci_read(unsigned int domain, unsigned int bus,
		       unsigned int devfn, int where, int size, u32 *value)
{
	return mk_pci_raw_read(domain, bus, devfn, where, size, value);
}

static int mk_pci_write(unsigned int domain, unsigned int bus,
			unsigned int devfn, int where, int size, u32 value)
{
	return mk_pci_raw_write(domain, bus, devfn, where, size, value);
}

static int mk_pci_ext_read(unsigned int domain, unsigned int bus,
			   unsigned int devfn, int where, int size, u32 *value)
{
	return mk_pci_raw_read(domain, bus, devfn, where, size, value);
}

static int mk_pci_ext_write(unsigned int domain, unsigned int bus,
			    unsigned int devfn, int where, int size, u32 value)
{
	return mk_pci_raw_write(domain, bus, devfn, where, size, value);
}

static const struct pci_raw_ops mk_pci_filtered_raw_ops = {
	.read = mk_pci_read,
	.write = mk_pci_write,
};

static const struct pci_raw_ops mk_pci_filtered_raw_ext_ops = {
	.read = mk_pci_ext_read,
	.write = mk_pci_ext_write,
};

static int mk_mmcfg_snapshot_region(const struct pci_mmcfg_region *region,
				    void *data)
{
	struct mk_mmcfg_snapshot *snapshot = data;
	const struct mk_pci_device *device;

	list_for_each_entry(device, &snapshot->instance->pci_devices, list) {
		if (device->domain != region->segment ||
		    device->bus < region->start_bus ||
		    device->bus > region->end_bus)
			continue;
		if (mk_mmcfg_snapshot_contains(snapshot, device->domain,
					       device->bus))
			continue;
		if (snapshot->count == snapshot->capacity)
			return -ENOSPC;

		snapshot->bridges[snapshot->count].segment = region->segment;
		snapshot->bridges[snapshot->count].bus_start = device->bus;
		snapshot->bridges[snapshot->count].bus_end = device->bus;
		snapshot->bridges[snapshot->count].ecam_base = region->address;
		pr_info("Multikernel publishing synthetic PCI root %04x:%02x ECAM base %#llx\n",
			region->segment, device->bus,
			(unsigned long long)region->address);
		snapshot->count++;
	}

	return 0;
}

int mk_arch_snapshot_pci_host_bridges(const struct mk_instance *instance,
				      struct mk_pci_host_bridge *bridges,
				      size_t capacity)
{
	const struct mk_pci_device *device;
	struct mk_mmcfg_snapshot snapshot = {
		.instance = instance,
		.bridges = bridges,
		.capacity = capacity,
	};
	int ret;

	if (!instance || !bridges || !capacity ||
	    !instance->pci_devices_valid || !instance->pci_device_count)
		return 0;

	ret = pci_mmcfg_walk_regions(mk_mmcfg_snapshot_region, &snapshot);
	if (ret)
		return ret;

	list_for_each_entry(device, &instance->pci_devices, list) {
		if (mk_mmcfg_snapshot_contains(&snapshot, device->domain,
					       device->bus))
			continue;
		pr_err("Multikernel has no ECAM window for assigned PCI bus %04x:%02x\n",
		       device->domain, device->bus);
		return -ENOENT;
	}

	return snapshot.count;
}

static int __init x86_multikernel_pci_arch_init(void)
{
	const struct mk_pci_host_bridge *bridge;
	int bridge_count = 0;

	if (!root_instance || !root_instance->pci_host_bridges_valid ||
	    !root_instance->pci_host_bridge_count) {
		pr_err("Multikernel has no restored PCI host bridge metadata\n");
		return 0;
	}
	mk_pci_host_instance = mk_instance_find(0);
	if (!mk_pci_host_instance) {
		pr_err("Multikernel has no restored host instance for PCI control\n");
		return 0;
	}
	if (!list_empty(&pci_mmcfg_list)) {
		pr_err("Multikernel PCI host bridge list was not empty before restore\n");
		return 0;
	}

	list_for_each_entry(bridge, &root_instance->pci_host_bridges, list) {
		if (bridge->bus_start != bridge->bus_end) {
			pr_err("Multikernel PCI root %04x:[%02x-%02x] exposes shared bridge topology\n",
			       bridge->segment, bridge->bus_start,
			       bridge->bus_end);
			return 0;
		}
		bridge_count++;
	}
	if (bridge_count != root_instance->pci_host_bridge_count) {
		pr_err("Multikernel PCI root count mismatch: expected %d, restored %d\n",
		       root_instance->pci_host_bridge_count, bridge_count);
		return 0;
	}

	list_for_each_entry(bridge, &root_instance->pci_host_bridges, list) {
		if (!pci_mmconfig_add(bridge->segment, bridge->bus_start,
				      bridge->bus_end, bridge->ecam_base)) {
			pr_err("Multikernel failed to register synthetic PCI root %04x:%02x\n",
			       bridge->segment, bridge->bus_start);
			return 0;
		}
		pr_notice("Multikernel restored synthetic PCI root %04x:%02x ECAM base %#llx\n",
			  bridge->segment, bridge->bus_start,
			  (unsigned long long)bridge->ecam_base);
	}
	if (!pci_mmcfg_arch_init()) {
		pr_err("Multikernel failed to map restored PCI ECAM windows\n");
		return 0;
	}
	if (mk_register_msg_handler(MK_MSG_PCI, mk_pci_cfg_response_handler,
				    NULL)) {
		pr_err("Multikernel failed to register PCI control-plane response handler\n");
		return 0;
	}

	raw_pci_ops = &pci_mmcfg;
	raw_pci_ext_ops = &pci_mmcfg;
	raw_pci_ops = &mk_pci_filtered_raw_ops;
	raw_pci_ext_ops = &mk_pci_filtered_raw_ext_ops;
	mk_pci_roots_ready = true;
	pr_notice("Multikernel selected filtered ECAM for PCI config access\n");

	return 0;
}

static int __init mk_pci_scan_root(const struct mk_pci_host_bridge *bridge)
{
	struct pci_sysdata *sd;
	struct pci_bus *bus;
	LIST_HEAD(resources);

	if (bridge->segment && !pci_domains_supported) {
		pr_err("Multikernel cannot scan PCI root %04x:%02x without domain support\n",
		       bridge->segment, bridge->bus_start);
		return -EOPNOTSUPP;
	}
	if (pci_find_bus(bridge->segment, bridge->bus_start)) {
		pr_err("Multikernel PCI root %04x:%02x already exists\n",
		       bridge->segment, bridge->bus_start);
		return -EEXIST;
	}

	sd = kzalloc(sizeof(*sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;
	sd->domain = bridge->segment;
	sd->node = x86_pci_root_bus_node(bridge->bus_start);
	x86_pci_root_bus_resources(bridge->bus_start, &resources);
	bus = pci_scan_root_bus(NULL, bridge->bus_start, &pci_root_ops, sd,
				&resources);
	if (!bus) {
		pci_free_resource_list(&resources);
		kfree(sd);
		return -ENOMEM;
	}
	pci_bus_add_devices(bus);
	pr_notice("Multikernel scanned synthetic PCI root %04x:%02x\n",
		  bridge->segment, bridge->bus_start);
	return 0;
}

static int __init x86_multikernel_pci_init(void)
{
	const struct mk_pci_host_bridge *bridge;
	int ret;

	if (!root_instance)
		panic("Multikernel lost restored instance metadata");
	if (!root_instance->pci_device_count)
		return 0;
	if (!root_instance->pci_devices_valid || !mk_pci_roots_ready)
		panic("Multikernel synthetic PCI roots are unavailable");

	list_for_each_entry(bridge, &root_instance->pci_host_bridges, list) {
		ret = mk_pci_scan_root(bridge);
		if (ret)
			panic("Multikernel failed to scan synthetic PCI root %04x:%02x: %d",
			      bridge->segment, bridge->bus_start, ret);
	}
	if (atomic64_read(&mk_pci_cfg_count)) {
		u64 count = atomic64_read(&mk_pci_cfg_count);

		pr_notice("Multikernel PCI control plane: %llu config requests, average %llu ns, max %llu ns\n",
			  count, atomic64_read(&mk_pci_cfg_total_ns) / count,
			  atomic64_read(&mk_pci_cfg_max_ns));
	}

	/* Suppress legacy bus 0 probing after every assigned root is present. */
	return 0;
}
#else
int mk_arch_snapshot_pci_host_bridges(const struct mk_instance *instance,
				      struct mk_pci_host_bridge *bridges,
				      size_t capacity)
{
	return 0;
}

static int __init x86_multikernel_pci_arch_init(void)
{
	pr_err("Multikernel PCI requires CONFIG_PCI_MMCONFIG\n");
	return 0;
}

static int __init x86_multikernel_pci_init(void)
{
	return 0;
}
#endif

void __init x86_multikernel_pci_platform_init(void)
{
	pci_probe = PCI_PROBE_MMCONF | PCI_PROBE_NOEARLY;
	x86_init.pci.arch_init = x86_multikernel_pci_arch_init;
	x86_init.pci.init = x86_multikernel_pci_init;
}
