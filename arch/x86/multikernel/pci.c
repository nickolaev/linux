// SPDX-License-Identifier: GPL-2.0-only
/*
 * x86 PCI support for multikernel spawn kernels.
 *
 * This file owns the x86-specific transport of PCI ECAM windows. Generic
 * MMCONFIG code only provides an iterator over its region list.
 */

#include <linux/init.h>
#include <linux/multikernel.h>
#include <linux/pci.h>

#include <asm/multikernel.h>
#include <asm/pci_x86.h>
#include <asm/x86_init.h>

struct mk_mmcfg_snapshot {
	const struct mk_instance *instance;
	struct mk_pci_host_bridge *bridges;
	size_t capacity;
	size_t count;
};

static struct pci_ops mk_pci_native_ops;

static bool mk_pci_identity_read(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 *value)
{
	u16 vendor, device;
	u32 identity;
	u32 mask;

	if (where < PCI_VENDOR_ID || where + size > PCI_COMMAND ||
	    !mk_pci_get_assigned_identity(bus, devfn, &vendor, &device))
		return false;

	identity = vendor | (u32)device << 16;
	mask = size == sizeof(identity) ? ~0U : (1U << (size * 8)) - 1;
	*value = (identity >> (where * 8)) & mask;
	return true;
}

static int mk_pci_read(struct pci_bus *bus, unsigned int devfn, int where,
		       int size, u32 *value)
{
	if (!mk_pci_should_probe(bus, devfn, &mk_pci_native_ops)) {
		*value = ~0U;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	if (mk_pci_identity_read(bus, devfn, where, size, value))
		return PCIBIOS_SUCCESSFUL;

	return mk_pci_native_ops.read(bus, devfn, where, size, value);
}

static int mk_pci_write(struct pci_bus *bus, unsigned int devfn, int where,
			int size, u32 value)
{
	if (!mk_pci_should_probe(bus, devfn, &mk_pci_native_ops))
		return PCIBIOS_DEVICE_NOT_FOUND;

	return mk_pci_native_ops.write(bus, devfn, where, size, value);
}

#ifdef CONFIG_PCI_MMCONFIG
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
		if (snapshot->count == snapshot->capacity)
			return -ENOSPC;

		snapshot->bridges[snapshot->count].segment = region->segment;
		snapshot->bridges[snapshot->count].bus_start = region->start_bus;
		snapshot->bridges[snapshot->count].bus_end = region->end_bus;
		snapshot->bridges[snapshot->count].ecam_base = region->address;
		pr_info("Multikernel publishing ECAM segment %04x [bus %02x-%02x] base %#llx\n",
			region->segment, region->start_bus, region->end_bus,
			(unsigned long long)region->address);
		snapshot->count++;
		break;
	}

	return 0;
}

int mk_arch_snapshot_pci_host_bridges(const struct mk_instance *instance,
				      struct mk_pci_host_bridge *bridges,
				      size_t capacity)
{
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
	return ret ?: snapshot.count;
}

static int __init x86_multikernel_pci_arch_init(void)
{
	const struct mk_pci_host_bridge *bridge;

	if (!root_instance || !root_instance->pci_host_bridges_valid ||
	    !root_instance->pci_host_bridge_count) {
		pr_err("Multikernel has no restored PCI host bridge metadata\n");
		return 1;
	}
	if (!list_empty(&pci_mmcfg_list)) {
		pr_err("Multikernel PCI host bridge list was not empty before restore\n");
		return 1;
	}

	list_for_each_entry(bridge, &root_instance->pci_host_bridges, list) {
		if (!pci_mmconfig_add(bridge->segment, bridge->bus_start,
				      bridge->bus_end, bridge->ecam_base))
			return 1;
		pr_notice("Multikernel restored ECAM segment %04x [bus %02x-%02x] base %#llx\n",
			  bridge->segment, bridge->bus_start, bridge->bus_end,
			  (unsigned long long)bridge->ecam_base);
	}
	if (!pci_mmcfg_arch_init()) {
		pr_err("Multikernel failed to map restored PCI ECAM windows\n");
		return 1;
	}

	raw_pci_ops = &pci_mmcfg;
	raw_pci_ext_ops = &pci_mmcfg;
	mk_pci_native_ops = pci_root_ops;
	pci_root_ops.read = mk_pci_read;
	pci_root_ops.write = mk_pci_write;
	pr_notice("Multikernel selected ECAM for PCI config access\n");

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
	return 1;
}
#endif

void __init x86_multikernel_pci_platform_init(void)
{
	pci_probe = PCI_PROBE_MMCONF | PCI_PROBE_NOEARLY;
	x86_init.pci.arch_init = x86_multikernel_pci_arch_init;
	x86_init.pci.init = pci_legacy_init;
}
