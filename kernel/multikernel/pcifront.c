// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Multikernel Technologies, Inc. All rights reserved
 *
 * The PCI buses of a spawn kernel, with config space reached through
 * the kernel that lent the devices (pciback.c).
 *
 * The spawn's tree describes each bus that holds a device of its own as
 * a root bus with only those devices on it. Config space accesses on
 * these buses are requests over the message ring: the lending kernel
 * checks that the function was lent, filters the access and makes it.
 * This kernel never generates a config cycle itself, so it cannot touch
 * a function it was not given, nor share a config mechanism with
 * another kernel.
 *
 * The PCI core calls config accessors under its raw lock with interrupts
 * off, so the wait for an answer polls the ring. Config space is only
 * touched to probe, reset, power manage and recover a device, and a
 * running device costs no requests at all: its BARs, its DMA and its
 * MSI-X vectors are direct.
 */

#define pr_fmt(fmt) "mk_pcifront: " fmt

#include <linux/multikernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/slab.h>

#include "internal.h"

#define MK_PCIFRONT_TIMEOUT_US	(10 * USEC_PER_MSEC)

/* The private part of each host bridge, one per bus */
struct mk_pcifront_bus {
	struct list_head list;
	u16 domain;
	u8 busnr;
};

/* Only added to, before anything can look */
static LIST_HEAD(mk_pcifront_buses);

static struct mk_pcifront_bus *mk_pcifront_of(struct pci_bus *bus)
{
	return pci_host_bridge_priv(to_pci_host_bridge(bus->bridge));
}

static int mk_pcifront_access(struct mk_pcifront_bus *fb, unsigned int devfn,
			      int where, int size, u32 *val, u32 subtype)
{
	struct mk_pci_cfg_payload req = {
		.seq = mk_request_seq(),
		.sender_instance_id = mk_self->id,
		.domain = fb->domain,
		.bus = fb->busnr,
		.devfn = devfn,
		.where = where,
		.size = size,
		.val = *val,
	};
	struct mk_pci_cfg_payload ack;
	int ret;

	ret = mk_request_sync(host_instance, MK_MSG_PCI, subtype, req.seq,
			      &req, sizeof(req), &ack, sizeof(ack),
			      MK_PCIFRONT_TIMEOUT_US);
	if (ret) {
		pr_err_ratelimited("%04x:%02x:%02x.%x: no answer for config offset %#x: %d\n",
				   fb->domain, fb->busnr, PCI_SLOT(devfn),
				   PCI_FUNC(devfn), where, ret);
		if (subtype == MK_PCI_CFG_READ)
			*val = ~0U;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (subtype == MK_PCI_CFG_READ)
		*val = ack.val;
	return ack.result;
}

static int mk_pcifront_read(struct pci_bus *bus, unsigned int devfn,
			    int where, int size, u32 *val)
{
	return mk_pcifront_access(mk_pcifront_of(bus), devfn, where, size, val,
				  MK_PCI_CFG_READ);
}

static int mk_pcifront_write(struct pci_bus *bus, unsigned int devfn,
			     int where, int size, u32 val)
{
	return mk_pcifront_access(mk_pcifront_of(bus), devfn, where, size,
				  &val, MK_PCI_CFG_WRITE);
}

/**
 * mk_pcifront_config - Config space access by address, outside the PCI core
 * @domain: PCI domain
 * @bus: Bus number
 * @devfn: Device and function
 * @where: Offset in config space
 * @size: 1, 2 or 4 bytes
 * @val: Value to write, or where the value read goes
 * @write: Whether to write
 *
 * For architecture code that reaches config space by address, such as
 * the x86 raw accessors. Only the buses of this kernel's tree exist.
 *
 * Returns: a PCIBIOS_* code, or -ENODEV for a bus this kernel does not have
 */
int mk_pcifront_config(unsigned int domain, unsigned int bus,
		       unsigned int devfn, int where, int size, u32 *val,
		       bool write)
{
	struct mk_pcifront_bus *fb;

	list_for_each_entry(fb, &mk_pcifront_buses, list) {
		if (fb->domain == domain && fb->busnr == bus)
			return mk_pcifront_access(fb, devfn, where, size, val,
						  write ? MK_PCI_CFG_WRITE :
							  MK_PCI_CFG_READ);
	}

	if (!write)
		*val = ~0U;
	return -ENODEV;
}

static struct pci_ops mk_pcifront_ops = {
	.read = mk_pcifront_read,
	.write = mk_pcifront_write,
};

/**
 * mk_arch_pcifront_prepare - Arch part of setting up a root bus
 * @bridge: The host bridge, not yet registered
 * @domain: Its PCI domain
 *
 * An architecture whose PCI code expects its own sysdata on a root bus
 * provides it here.
 */
int __weak mk_arch_pcifront_prepare(struct pci_host_bridge *bridge, int domain)
{
	return 0;
}

static void mk_pcifront_add_windows(struct device_node *np,
				    struct pci_host_bridge *bridge)
{
	struct of_pci_range_parser parser;
	struct of_pci_range range;
	struct resource *res;

	if (of_pci_range_parser_init(&parser, np))
		return;

	for_each_of_pci_range(&parser, &range) {
		if (!(range.flags & (IORESOURCE_IO | IORESOURCE_MEM)))
			continue;

		res = kzalloc_obj(*res, GFP_KERNEL);
		if (!res)
			return;
		if (of_pci_range_to_resource(&range, np, res)) {
			kfree(res);
			continue;
		}
		res->name = "PCI Bus multikernel";
		pci_add_resource_offset(&bridge->windows, res,
					res->start - range.pci_addr);
	}
}

static int __init mk_pcifront_add_bus(struct device_node *np)
{
	struct pci_host_bridge *bridge;
	struct resource_entry *entry;
	struct mk_pcifront_bus *fb;
	struct resource *bus_res;
	u32 domain = 0, range[2];
	int ret;

	of_property_read_u32(np, "linux,pci-domain", &domain);
	if (of_property_read_u32_array(np, "bus-range", range, 2)) {
		pr_warn("%pOF has no bus-range, skipped\n", np);
		return -EINVAL;
	}

	bridge = pci_alloc_host_bridge(sizeof(*fb));
	bus_res = kzalloc_obj(*bus_res, GFP_KERNEL);
	if (!bridge || !bus_res) {
		ret = -ENOMEM;
		goto err;
	}

	fb = pci_host_bridge_priv(bridge);
	fb->domain = domain;
	fb->busnr = range[0];

	bus_res->start = range[0];
	bus_res->end = range[1];
	bus_res->flags = IORESOURCE_BUS;
	bus_res->name = np->full_name;
	pci_add_resource(&bridge->windows, bus_res);
	mk_pcifront_add_windows(np, bridge);

	/* The bus binds to its node by identity, not by a bus number search */
	bridge->dev.of_node = of_node_get(np);
	bridge->busnr = range[0];
	bridge->ops = &mk_pcifront_ops;
	if (IS_ENABLED(CONFIG_PCI_DOMAINS_GENERIC))
		bridge->domain_nr = domain;

	ret = mk_arch_pcifront_prepare(bridge, domain);
	if (ret)
		goto err_node;

	ret = pci_scan_root_bus_bridge(bridge);
	if (ret)
		goto err_node;

	/* The host assigned the BARs and keeps them, so take them as they are */
	pci_bus_claim_resources(bridge->bus);

	list_add_tail(&fb->list, &mk_pcifront_buses);
	pci_bus_add_devices(bridge->bus);
	pr_info("Root bus %04x:%02x from %pOF\n", domain, range[0], np);
	return 0;

err_node:
	of_node_put(np);
	resource_list_for_each_entry(entry, &bridge->windows)
		kfree(entry->res);
	pci_free_host_bridge(bridge);
	return ret;
err:
	kfree(bus_res);
	if (bridge)
		pci_free_host_bridge(bridge);
	return ret;
}

/**
 * mk_pcifront_init - Create the PCI buses of a spawn kernel
 *
 * Runs once. An architecture may call it from where its PCI init order
 * wants the buses; it runs at subsys_initcall otherwise.
 *
 * Returns: 0, also when this kernel is not a spawn or has no PCI buses
 */
int __init mk_pcifront_init(void)
{
	static bool done;
	struct device_node *np;
	int scanned = 0;

	if (done || !mk_manifest_phys())
		return 0;
	done = true;

	if (!host_instance) {
		pr_err("No ring to the host, no PCI for this instance\n");
		return 0;
	}

	for_each_compatible_node(np, NULL, MK_PCI_HOST_BRIDGE)
		if (!mk_pcifront_add_bus(np))
			scanned++;

	if (!scanned)
		pr_info("No PCI buses described for this instance\n");
	return 0;
}

static int __init mk_pcifront_initcall(void)
{
	return mk_pcifront_init();
}
subsys_initcall(mk_pcifront_initcall);
