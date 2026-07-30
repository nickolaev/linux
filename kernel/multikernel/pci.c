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

static struct mk_pci_device *mk_pci_find_assigned(struct pci_bus *bus, int devfn)
{
	struct mk_pci_device *device;
	u16 domain = pci_domain_nr(bus);
	u8 slot = PCI_SLOT(devfn);
	u8 func = PCI_FUNC(devfn);

	if (!root_instance || !root_instance->dtb_data ||
	    !root_instance->pci_devices_valid)
		return NULL;

	list_for_each_entry(device, &root_instance->pci_devices, list) {
		if (device->domain == domain && device->bus == bus->number &&
		    device->slot == slot && device->func == func)
			return device;
	}

	return NULL;
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
