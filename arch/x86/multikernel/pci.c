// SPDX-License-Identifier: GPL-2.0-only
/*
 * x86 PCI support for multikernel spawn kernels.
 *
 * This file owns the x86-specific transport of PCI ECAM windows. Generic
 * MMCONFIG code only provides an iterator over its region list.
 */

#include <linux/init.h>
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

static const struct pci_raw_ops *mk_pci_native_raw_ops;
static const struct pci_raw_ops *mk_pci_native_raw_ext_ops;
static bool mk_pci_roots_ready;

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

static int mk_pci_raw_read(const struct pci_raw_ops *native,
			   unsigned int domain, unsigned int bus,
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

	return native->read(domain, bus, devfn, where, size, value);
}

static int mk_pci_raw_write(const struct pci_raw_ops *native,
			    unsigned int domain, unsigned int bus,
			    unsigned int devfn, int where, int size,
			    u32 value)
{
	if (!mk_pci_get_assigned_identity_bdf(domain, bus, devfn, NULL, NULL))
		return PCIBIOS_DEVICE_NOT_FOUND;

	return native->write(domain, bus, devfn, where, size, value);
}

static int mk_pci_read(unsigned int domain, unsigned int bus,
		       unsigned int devfn, int where, int size, u32 *value)
{
	return mk_pci_raw_read(mk_pci_native_raw_ops, domain, bus, devfn,
			       where, size, value);
}

static int mk_pci_write(unsigned int domain, unsigned int bus,
			unsigned int devfn, int where, int size, u32 value)
{
	return mk_pci_raw_write(mk_pci_native_raw_ops, domain, bus, devfn,
				where, size, value);
}

static int mk_pci_ext_read(unsigned int domain, unsigned int bus,
			   unsigned int devfn, int where, int size, u32 *value)
{
	return mk_pci_raw_read(mk_pci_native_raw_ext_ops, domain, bus, devfn,
			       where, size, value);
}

static int mk_pci_ext_write(unsigned int domain, unsigned int bus,
			    unsigned int devfn, int where, int size, u32 value)
{
	return mk_pci_raw_write(mk_pci_native_raw_ext_ops, domain, bus, devfn,
				where, size, value);
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

	raw_pci_ops = &pci_mmcfg;
	raw_pci_ext_ops = &pci_mmcfg;
	mk_pci_native_raw_ops = raw_pci_ops;
	mk_pci_native_raw_ext_ops = raw_pci_ext_ops;
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
