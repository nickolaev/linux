// SPDX-License-Identifier: GPL-2.0-only
/*
 * x86 PCI support for multikernel spawn kernels.
 *
 * Spawn kernels discover only assigned BDFs. Configuration accesses are
 * filtered here and become host-mediated once the RPC transport is installed.
 */
#include <linux/init.h>
#include <linux/multikernel.h>
#include <linux/panic.h>
#include <linux/pci.h>
#include <linux/topology.h>

#include <asm/pci_x86.h>
#include <asm/x86_init.h>

static struct pci_ops mk_pci_native_ops;
static bool mk_pci_roots_ready;

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

static int mk_pci_read(struct pci_bus *bus, unsigned int devfn, int where,
		       int size, u32 *value)
{
	u16 vendor, device;

	if (!mk_pci_get_assigned_identity(bus, devfn, &vendor, &device)) {
		*value = ~0U;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	if (mk_pci_identity_read(vendor, device, where, size, value))
		return PCIBIOS_SUCCESSFUL;
	return mk_pci_native_ops.read(bus, devfn, where, size, value);
}

static int mk_pci_write(struct pci_bus *bus, unsigned int devfn, int where,
			int size, u32 value)
{
	if (!mk_pci_get_assigned_identity(bus, devfn, NULL, NULL))
		return PCIBIOS_DEVICE_NOT_FOUND;
	return mk_pci_native_ops.write(bus, devfn, where, size, value);
}

static int __init x86_multikernel_pci_arch_init(void)
{
	if (!root_instance || !root_instance->pci_devices_valid)
		return 0;

	mk_pci_native_ops = pci_root_ops;
	pci_root_ops.read = mk_pci_read;
	pci_root_ops.write = mk_pci_write;
	mk_pci_roots_ready = true;
	pr_notice("Multikernel selected filtered PCI config access\n");
	return 0;
}

static struct pci_bus * __init mk_pci_get_root(u16 domain, u8 bus_number)
{
	struct resource_entry *window;
	struct pci_sysdata *sd;
	struct pci_bus *bus;
	bool has_busn_res = false;
	LIST_HEAD(resources);

	if (domain && !pci_domains_supported) {
		pr_err("Multikernel cannot scan PCI root %04x:%02x without domain support\n",
		       domain, bus_number);
		return ERR_PTR(-EOPNOTSUPP);
	}
	bus = pci_find_bus(domain, bus_number);
	if (bus)
		return bus;

	sd = kzalloc_obj(*sd, GFP_KERNEL);
	if (!sd)
		return ERR_PTR(-ENOMEM);
	sd->domain = domain;
	sd->node = x86_pci_root_bus_node(bus_number);
	x86_pci_root_bus_resources(bus_number, &resources);
	resource_list_for_each_entry(window, &resources) {
		if (window->res->flags & IORESOURCE_BUS) {
			has_busn_res = true;
			break;
		}
	}
	bus = pci_create_root_bus(NULL, bus_number, &pci_root_ops, sd,
				  &resources);
	if (!bus) {
		pci_free_resource_list(&resources);
		kfree(sd);
		return ERR_PTR(-ENOMEM);
	}
	if (!has_busn_res &&
	    !pci_bus_insert_busn_res(bus, bus_number, bus_number)) {
		pci_remove_root_bus(bus);
		kfree(sd);
		return ERR_PTR(-EBUSY);
	}
	pr_notice("Multikernel created synthetic PCI root %04x:%02x\n",
		  domain, bus_number);
	return bus;
}

static int __init x86_multikernel_pci_init(void)
{
	const struct mk_pci_device *device;
	struct pci_bus *bus;
	struct pci_dev *pdev;

	if (!root_instance)
		panic("Multikernel lost restored instance metadata");
	if (!root_instance->pci_device_count)
		return 0;
	if (!root_instance->pci_devices_valid || !mk_pci_roots_ready)
		panic("Multikernel assigned PCI inventory is unavailable");

	list_for_each_entry(device, &root_instance->pci_devices, list) {
		bus = mk_pci_get_root(device->domain, device->bus);
		if (IS_ERR(bus))
			panic("Multikernel failed to create synthetic PCI root %04x:%02x: %ld",
			      device->domain, device->bus, PTR_ERR(bus));
		pdev = pci_scan_single_device(bus,
					      PCI_DEVFN(device->slot, device->func));
		if (!pdev)
			panic("Multikernel failed to enumerate assigned PCI device %04x:%02x:%02x.%x",
			      device->domain, device->bus, device->slot,
			      device->func);
		pci_bus_add_devices(bus);
	}

	/* Suppress legacy bus 0 probing after every assigned function is present. */
	return 0;
}

void __init x86_multikernel_pci_platform_init(void)
{
	pci_probe = PCI_PROBE_NOEARLY;
	x86_init.pci.arch_init = x86_multikernel_pci_arch_init;
	x86_init.pci.init = x86_multikernel_pci_init;
}
