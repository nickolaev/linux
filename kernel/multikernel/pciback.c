// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Multikernel Technologies, Inc. All rights reserved
 *
 * Config space of the PCI functions lent to instances, served over the
 * message ring.
 *
 * An instance never generates a config cycle itself: it asks the kernel
 * that lent it the function, which checks that the function is the
 * instance's and makes the access with its own accessors, quirks
 * included. Nothing else on the fabric is reachable that way, the
 * bridges above the function included, and no two kernels ever share a
 * config mechanism such as the 0xCF8 port pair.
 *
 * Only config space goes this way. BARs, DMA and MSIs stay direct, so
 * none of this is on the data path of a running device.
 *
 * Requests are answered from the doorbell interrupt, which is what lets
 * an instance wait for the answer with interrupts off. A kernel that is
 * itself an instance reaches config space over the ring too, which it
 * cannot do from a message handler, so it answers from a work item.
 */

#define pr_fmt(fmt) "mk_pciback: " fmt

#include <linux/multikernel.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "internal.h"

/* A function an instance may reach, while the instance runs */
struct mk_pciback_dev {
	struct list_head list;
	struct mk_instance *instance;
	struct pci_dev *pdev;
};

static LIST_HEAD(mk_pciback_devs);
static DEFINE_RAW_SPINLOCK(mk_pciback_lock);

struct mk_pciback_work {
	struct work_struct work;
	struct mk_pci_cfg_payload req;
	u32 subtype;
};

static struct mk_pciback_dev *mk_pciback_find(int instance_id, u16 domain,
					      u8 bus, u8 devfn)
{
	struct mk_pciback_dev *d;

	lockdep_assert_held(&mk_pciback_lock);

	list_for_each_entry(d, &mk_pciback_devs, list) {
		if (d->instance->id == instance_id &&
		    pci_domain_nr(d->pdev->bus) == domain &&
		    d->pdev->bus->number == bus && d->pdev->devfn == devfn)
			return d;
	}
	return NULL;
}

static bool mk_pciback_valid(const struct mk_pci_cfg_payload *req)
{
	if (req->size != 1 && req->size != 2 && req->size != 4)
		return false;
	return req->where % req->size == 0 &&
	       req->where + req->size <= PCI_CFG_SPACE_EXP_SIZE;
}

static int mk_pciback_read(struct pci_dev *pdev, int where, int size, u32 *val)
{
	u16 w;
	u8 b;
	int ret;

	switch (size) {
	case 1:
		ret = pci_read_config_byte(pdev, where, &b);
		*val = b;
		break;
	case 2:
		ret = pci_read_config_word(pdev, where, &w);
		*val = w;
		break;
	default:
		ret = pci_read_config_dword(pdev, where, val);
		break;
	}
	return ret;
}

static int mk_pciback_write(struct pci_dev *pdev, int where, int size, u32 val)
{
	switch (size) {
	case 1:
		return pci_write_config_byte(pdev, where, val);
	case 2:
		return pci_write_config_word(pdev, where, val);
	default:
		return pci_write_config_dword(pdev, where, val);
	}
}

static void mk_pciback_do(struct pci_dev *pdev, u32 subtype,
			  struct mk_pci_cfg_payload *ack)
{
	if (!mk_pciback_valid(ack)) {
		ack->result = PCIBIOS_BAD_REGISTER_NUMBER;
	} else if (subtype == MK_PCI_CFG_READ) {
		ack->result = mk_pciback_read(pdev, ack->where, ack->size,
					      &ack->val);
	} else {
		ack->result = mk_pciback_write(pdev, ack->where, ack->size,
					       ack->val);
	}

	if (subtype == MK_PCI_CFG_READ && ack->result)
		ack->val = ~0U;
}

static void mk_pciback_answer(struct mk_instance *instance,
			      struct mk_pci_cfg_payload *ack)
{
	ack->sender_instance_id = mk_self->id;
	mk_send_message_to(instance, MK_MSG_PCI, MK_PCI_CFG_ACK, ack,
			   sizeof(*ack));
}

/*
 * The slow way: the requester is not known here (it was lent nothing),
 * or this kernel's own config access goes over the ring.
 */
static void mk_pciback_work_fn(struct work_struct *work)
{
	struct mk_pciback_work *w = container_of(work, struct mk_pciback_work, work);
	struct mk_pci_cfg_payload *ack = &w->req;
	struct mk_instance *instance;
	struct mk_pciback_dev *d;
	struct pci_dev *pdev = NULL;

	instance = mk_instance_find(ack->sender_instance_id);
	if (!instance)
		goto out;

	scoped_guard(raw_spinlock_irqsave, &mk_pciback_lock) {
		d = mk_pciback_find(instance->id, ack->domain, ack->bus,
				    ack->devfn);
		if (d)
			pdev = pci_dev_get(d->pdev);
	}

	if (pdev) {
		mk_pciback_do(pdev, w->subtype, ack);
		pci_dev_put(pdev);
	} else {
		ack->result = PCIBIOS_DEVICE_NOT_FOUND;
		ack->val = ~0U;
	}

	mk_pciback_answer(instance, ack);
	mk_instance_put(instance);
out:
	kfree(w);
}

static void mk_pciback_defer(const struct mk_pci_cfg_payload *req, u32 subtype)
{
	struct mk_pciback_work *w = kzalloc(sizeof(*w), GFP_ATOMIC);

	if (!w)
		return;
	w->req = *req;
	w->subtype = subtype;
	INIT_WORK(&w->work, mk_pciback_work_fn);
	schedule_work(&w->work);
}

static void mk_pciback_request(const struct mk_pci_cfg_payload *req,
			       u32 subtype)
{
	struct mk_pci_cfg_payload ack = *req;
	struct mk_pciback_dev *d;

	if (mk_manifest_phys()) {
		mk_pciback_defer(req, subtype);
		return;
	}

	guard(raw_spinlock_irqsave)(&mk_pciback_lock);

	d = mk_pciback_find(req->sender_instance_id, req->domain, req->bus,
			    req->devfn);
	if (!d) {
		mk_pciback_defer(req, subtype);
		return;
	}

	mk_pciback_do(d->pdev, subtype, &ack);
	mk_pciback_answer(d->instance, &ack);
}

static void mk_pciback_msg_handler(u32 msg_type, u32 subtype, void *payload,
				   u32 payload_len, void *ctx)
{
	if (payload_len < sizeof(struct mk_pci_cfg_payload))
		return;

	switch (subtype) {
	case MK_PCI_CFG_READ:
	case MK_PCI_CFG_WRITE:
		mk_pciback_request(payload, subtype);
		break;
	case MK_PCI_CFG_ACK:
		mk_request_answer(((struct mk_pci_cfg_payload *)payload)->seq,
				  payload, sizeof(struct mk_pci_cfg_payload));
		break;
	}
}

/**
 * mk_pciback_lend_one - Let a running instance reach a function's config space
 * @instance: The instance
 * @domain: PCI domain
 * @bus: PCI bus
 * @devfn: PCI device and function
 *
 * Returns: 0 on success, or a negative error code
 */
int mk_pciback_lend_one(struct mk_instance *instance, u16 domain, u8 bus,
			u8 devfn)
{
	struct mk_pciback_dev *d;
	struct pci_dev *pdev;

	pdev = pci_get_domain_bus_and_slot(domain, bus, devfn);
	if (!pdev) {
		pr_warn("Instance %d: no PCI function %04x:%02x:%02x.%x here to lend\n",
			instance->id, domain, bus, PCI_SLOT(devfn),
			PCI_FUNC(devfn));
		return -ENODEV;
	}

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d) {
		pci_dev_put(pdev);
		return -ENOMEM;
	}

	/* Config cycles to a function in D3cold come back as all ones */
	pm_runtime_get_sync(&pdev->dev);

	d->pdev = pdev;
	d->instance = instance;
	mk_instance_get(instance);

	scoped_guard(raw_spinlock_irqsave, &mk_pciback_lock)
		list_add(&d->list, &mk_pciback_devs);
	return 0;
}

static void mk_pciback_free(struct list_head *gone)
{
	struct mk_pciback_dev *d, *tmp;

	list_for_each_entry_safe(d, tmp, gone, list) {
		pm_runtime_put(&d->pdev->dev);
		pci_dev_put(d->pdev);
		mk_instance_put(d->instance);
		kfree(d);
	}
}

/**
 * mk_pciback_unlend_one - Take back one function from an instance
 */
void mk_pciback_unlend_one(struct mk_instance *instance, u16 domain, u8 bus,
			   u8 devfn)
{
	struct mk_pciback_dev *d;
	LIST_HEAD(gone);

	scoped_guard(raw_spinlock_irqsave, &mk_pciback_lock) {
		d = mk_pciback_find(instance->id, domain, bus, devfn);
		if (d)
			list_move(&d->list, &gone);
	}
	mk_pciback_free(&gone);
}

/**
 * mk_pciback_unlend - Take back every function an instance could reach
 * @instance: The instance, which no longer runs
 */
void mk_pciback_unlend(struct mk_instance *instance)
{
	struct mk_pciback_dev *d, *tmp;
	LIST_HEAD(gone);

	scoped_guard(raw_spinlock_irqsave, &mk_pciback_lock) {
		list_for_each_entry_safe(d, tmp, &mk_pciback_devs, list) {
			if (d->instance == instance)
				list_move(&d->list, &gone);
		}
	}
	mk_pciback_free(&gone);
}

/**
 * mk_pciback_lend - Let an instance about to start reach its functions
 * @instance: The instance
 *
 * Returns: 0 on success, or a negative error code, in which case nothing
 * is lent and the instance must not be started
 */
int mk_pciback_lend(struct mk_instance *instance)
{
	struct mk_pci_device *dev;
	int ret;

	mk_pciback_unlend(instance);

	if (!instance->pci_devices_valid)
		return 0;

	list_for_each_entry(dev, &instance->pci_devices, list) {
		ret = mk_pciback_lend_one(instance, dev->domain, dev->bus,
					  PCI_DEVFN(dev->slot, dev->func));
		if (ret) {
			mk_pciback_unlend(instance);
			return ret;
		}
	}
	return 0;
}

static int __init mk_pciback_init(void)
{
	return mk_register_msg_handler(MK_MSG_PCI, mk_pciback_msg_handler, NULL);
}
fs_initcall(mk_pciback_init);
