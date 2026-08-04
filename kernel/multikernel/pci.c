// SPDX-License-Identifier: GPL-2.0-only
/*
 * Multikernel PCI assignment policy
 *
 * Keeps assigned-device discovery, identity presentation, bridge traversal,
 * and resource restoration independent from the manifest that populated
 * the current instance.
 */

#include <linux/multikernel.h>
#include <linux/pci.h>

#include "internal.h"

static struct mk_pci_device *
mk_pci_find_assigned_bdf(u16 domain, u8 bus, u8 devfn)
{
	struct mk_pci_device *device;
	u8 slot = PCI_SLOT(devfn);
	u8 func = PCI_FUNC(devfn);

	if (!root_instance || !root_instance->dtb_data ||
	    !root_instance->pci_devices_valid)
		return NULL;

	list_for_each_entry(device, &root_instance->pci_devices, list) {
		if (device->domain == domain && device->bus == bus &&
		    device->slot == slot && device->func == func)
			return device;
	}

	return NULL;
}

static struct mk_pci_device *mk_pci_find_assigned(struct pci_bus *bus, int devfn)
{
	return mk_pci_find_assigned_bdf(pci_domain_nr(bus), bus->number, devfn);
}

/**
 * mk_pci_get_assigned_identity_bdf - Get an assigned function's identity
 * @domain: PCI domain number
 * @bus: PCI bus number
 * @devfn: device/function number
 * @vendor: assigned Vendor ID
 * @device_id: assigned Device ID
 *
 * Returns: true when assignment metadata contains an exact location match.
 */
bool mk_pci_get_assigned_identity_bdf(unsigned int domain, unsigned int bus,
				      unsigned int devfn, u16 *vendor,
				      u16 *device_id)
{
	struct mk_pci_device *device;

	if (domain != (u16)domain || bus != (u8)bus || devfn != (u8)devfn)
		return false;
	device = mk_pci_find_assigned_bdf(domain, bus, devfn);

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
