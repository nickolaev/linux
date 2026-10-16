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
 *
 * A spawn outlives a host that crashes. Its devices keep running, but a
 * dead host answers nothing: a request that times out fails like an
 * access to an absent function, and nothing more. Whether the host is
 * dead is not decided here. User space decides, and acts on it by
 * fencing the host; once the fence has confirmed every CPU of the host
 * parked, accesses go straight to the ECAM window of the bus, the reg of
 * its node, which is mapped at boot and unused until then, or fail at
 * once where there is none. That is safe only because a fenced host no
 * longer generates config cycles itself; the 0xCF8 ports, which other
 * surviving kernels may use, are never touched.
 */

#define pr_fmt(fmt) "mk_pcifront: " fmt

#include <linux/multikernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include "internal.h"

#define MK_PCIFRONT_TIMEOUT_US	(10 * USEC_PER_MSEC)

/* The private part of each host bridge, one per bus */
struct mk_pcifront_bus {
	struct list_head list;
	u16 domain;
	u8 busnr;
	phys_addr_t ecam_phys;
	void __iomem *ecam;	/* For when the host is fenced, or NULL */
};

/* Only added to, before anything can look */
static LIST_HEAD(mk_pcifront_buses);

static bool mk_pcifront_direct_access;

static struct mk_pcifront_bus *mk_pcifront_of(struct pci_bus *bus)
{
	return pci_host_bridge_priv(to_pci_host_bridge(bus->bridge));
}

/* One request to the host. Returns 0 once it answered, with @result set */
static int mk_pcifront_ask(struct mk_pcifront_bus *fb, unsigned int devfn,
			   int where, int size, u32 *val, u32 subtype,
			   int *result)
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
	if (ret)
		return ret;

	if (subtype == MK_PCI_CFG_READ)
		*val = ack.val;
	*result = ack.result;
	return 0;
}

static int mk_pcifront_direct(struct mk_pcifront_bus *fb, unsigned int devfn,
			      int where, int size, u32 *val, u32 subtype)
{
	void __iomem *addr;

	if (!fb->ecam) {
		if (subtype == MK_PCI_CFG_READ)
			*val = ~0U;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	addr = fb->ecam + (devfn << 12) + where;
	if (subtype == MK_PCI_CFG_READ) {
		if (size == 1)
			*val = readb(addr);
		else if (size == 2)
			*val = readw(addr);
		else
			*val = readl(addr);
	} else {
		if (size == 1)
			writeb(*val, addr);
		else if (size == 2)
			writew(*val, addr);
		else
			writel(*val, addr);
	}
	return PCIBIOS_SUCCESSFUL;
}

/**
 * mk_pcifront_host_fenced - The host can no longer touch config space
 *
 * Called once a fence has confirmed every CPU of the host parked. From
 * then on this kernel reaches the config space of its devices itself.
 */
void mk_pcifront_host_fenced(void)
{
	struct mk_pcifront_bus *fb;
	bool direct = false;

	if (READ_ONCE(mk_pcifront_direct_access) ||
	    list_empty(&mk_pcifront_buses))
		return;
	WRITE_ONCE(mk_pcifront_direct_access, true);

	list_for_each_entry(fb, &mk_pcifront_buses, list)
		direct |= !!fb->ecam;
	pr_info("Host fenced: config space is reached %s\n",
		direct ? "directly through ECAM" : "nowhere");
}

static int mk_pcifront_access(struct mk_pcifront_bus *fb, unsigned int devfn,
			      int where, int size, u32 *val, u32 subtype)
{
	int result, ret;

	if (READ_ONCE(mk_pcifront_direct_access))
		return mk_pcifront_direct(fb, devfn, where, size, val, subtype);

	ret = mk_pcifront_ask(fb, devfn, where, size, val, subtype, &result);
	if (!ret)
		return result;

	pr_err_ratelimited("%04x:%02x:%02x.%x: no answer for config offset %#x: %d\n",
			   fb->domain, fb->busnr, PCI_SLOT(devfn),
			   PCI_FUNC(devfn), where, ret);
	if (subtype == MK_PCI_CFG_READ)
		*val = ~0U;
	return PCIBIOS_DEVICE_NOT_FOUND;
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

/**
 * mk_pcifront_ecam - The ECAM window of a bus of this kernel's tree
 * @domain: PCI domain
 * @bus: Bus number
 * @size: Set to the window's size
 *
 * Passed on to the tree of an instance of this kernel, for when both its
 * host and this kernel are fenced.
 *
 * Returns: the window's physical address, or 0 if the tree has none
 */
u64 mk_pcifront_ecam(unsigned int domain, unsigned int bus, u64 *size)
{
	struct mk_pcifront_bus *fb;

	list_for_each_entry(fb, &mk_pcifront_buses, list) {
		if (fb->domain == domain && fb->busnr == bus && fb->ecam) {
			*size = SZ_1M;
			return fb->ecam_phys;
		}
	}
	return 0;
}

/* Mapped now: nothing can be mapped under the PCI core's lock, where it is used */
static void __init mk_pcifront_map_ecam(struct device_node *np,
					struct mk_pcifront_bus *fb)
{
	struct resource ecam;

	if (of_address_to_resource(np, 0, &ecam))
		return;

	if (resource_size(&ecam) < SZ_1M) {
		pr_warn("%pOF: ECAM window %pR is too small\n", np, &ecam);
		return;
	}

	fb->ecam = ioremap(ecam.start, SZ_1M);
	if (fb->ecam)
		fb->ecam_phys = ecam.start;
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
	mk_pcifront_map_ecam(np, fb);

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
