// SPDX-License-Identifier: GPL-2.0-only
/*
 * x86 PCI support for multikernel spawn kernels.
 *
 * Spawn kernels discover only assigned BDFs. Configuration accesses are
 * filtered here and become host-mediated once the RPC transport is installed.
 */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ktime.h>
#include <linux/multikernel.h>
#include <linux/panic.h>
#include <linux/pci.h>
#include <linux/topology.h>

#include <asm/pci_x86.h>
#include <asm/x86_init.h>

static bool mk_pci_roots_ready;
#define MK_PCI_ENUM_RETRIES	3
#define MK_PCI_ENUM_RETRY_MS	20
static atomic64_t mk_pci_request_id = ATOMIC64_INIT(0);
static atomic64_t mk_pci_cfg_count = ATOMIC64_INIT(0);
static atomic64_t mk_pci_cfg_total_ns = ATOMIC64_INIT(0);
static atomic64_t mk_pci_cfg_max_ns = ATOMIC64_INIT(0);
static struct mk_instance *mk_pci_host_instance;

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
	struct mk_reply_handle reply;
	s32 status;
	u32 response_value;
	u64 start = ktime_get_mono_fast_ns();
	int ret;

	ret = mk_reply_reserve(root_instance->ipi_data, MK_REPLY_PCI_CFG,
			       request.request_id, &reply);
	if (ret)
		goto out_error;
	request.reply_slot = reply.slot;
	request.reply_generation = reply.generation;
	ret = mk_send_message(0, MK_MSG_PCI, MK_PCI_CFG_REQUEST,
			      &request, sizeof(request));
	if (ret) {
		mk_reply_release(root_instance->ipi_data, &reply);
		goto out_error;
	}
	ret = mk_reply_wait_atomic(root_instance->ipi_data, &reply, 20000,
				   &status, &response_value);
	if (ret)
		goto out_error;
	if (status < 0) {
		ret = status;
		goto out_error;
	}
	if (!write)
		*value = response_value;
	mk_pci_record_latency(start);
	return status;

out_error:
	if (ret < 0) {
		pr_err_ratelimited("Multikernel PCI config request timed out or failed to send: %d\n",
				   ret);
		return PCIBIOS_SET_FAILED;
	}
	return PCIBIOS_SET_FAILED;
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

static const struct pci_raw_ops mk_pci_filtered_raw_ops = {
	.read = mk_pci_raw_read,
	.write = mk_pci_raw_write,
};
static int __init x86_multikernel_pci_arch_init(void)
{
	if (!root_instance || !root_instance->pci_devices_valid)
		return 0;
	mk_pci_host_instance = mk_instance_find(0);
	if (!mk_pci_host_instance) {
		pr_err("Multikernel has no restored host instance for PCI control\n");
		return 0;
	}
	raw_pci_ops = &mk_pci_filtered_raw_ops;
	raw_pci_ext_ops = &mk_pci_filtered_raw_ops;
	mk_pci_roots_ready = true;
	pr_notice("Multikernel selected filtered raw PCI config access\n");
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

static struct pci_dev * __init
mk_pci_scan_assigned_device(struct pci_bus *bus, unsigned int devfn)
{
	struct pci_dev *pdev;
	unsigned int attempt;

	for (attempt = 0; attempt < MK_PCI_ENUM_RETRIES; attempt++) {
		pdev = pci_scan_single_device(bus, devfn);
		if (pdev)
			return pdev;
		if (attempt + 1 < MK_PCI_ENUM_RETRIES)
			msleep(MK_PCI_ENUM_RETRY_MS);
	}

	return NULL;
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
		pdev = mk_pci_scan_assigned_device(bus,
						   PCI_DEVFN(device->slot,
							     device->func));
		if (!pdev)
			panic("Multikernel failed to enumerate assigned PCI device %04x:%02x:%02x.%x after %u attempts",
			      device->domain, device->bus, device->slot,
			      device->func, MK_PCI_ENUM_RETRIES);
		pci_bus_add_devices(bus);
	}
	if (atomic64_read(&mk_pci_cfg_count)) {
		u64 count = atomic64_read(&mk_pci_cfg_count);

		pr_notice("Multikernel PCI control plane: %llu config requests, average %llu ns, max %llu ns\n",
			  count, atomic64_read(&mk_pci_cfg_total_ns) / count,
			  atomic64_read(&mk_pci_cfg_max_ns));
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
