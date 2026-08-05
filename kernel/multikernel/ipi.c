// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Multikernel Technologies, Inc. All rights reserved
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/percpu.h>
#include <linux/spinlock.h>
#include <linux/multikernel.h>
#include <linux/kexec.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include "internal.h"

/* Callback management */
static struct mk_ipi_handler *mk_handlers;
static raw_spinlock_t mk_handlers_lock = __RAW_SPIN_LOCK_UNLOCKED(mk_handlers_lock);

static void mk_ipi_drain_ring(void);

/*
 * head is an allocation cursor, not the consumer-visible publication point.
 * A producer killed while WRITING can strand only its claimed slot; READY
 * slots elsewhere remain visible to the consumer and to the NMI shutdown scan.
 */
static int mk_ipi_ring_claim_slot(struct mk_ipi_ring *ring,
				  struct mk_ipi_data **slot_out)
{
	struct mk_ipi_data *slot;
	unsigned int scanned = 0;
	unsigned int idx;
	int head;
	int next;

	while (scanned < MK_IPI_RING_SIZE) {
		head = atomic_read(&ring->head);
		idx = head & (MK_IPI_RING_SIZE - 1);
		next = (idx + 1) & (MK_IPI_RING_SIZE - 1);

		if (atomic_cmpxchg(&ring->head, head, next) != head) {
			cpu_relax();
			continue;
		}

		slot = &ring->entries[idx];
		if (atomic_cmpxchg(&slot->state, MK_IPI_SLOT_EMPTY,
				   MK_IPI_SLOT_WRITING) == MK_IPI_SLOT_EMPTY) {
			*slot_out = slot;
			return 0;
		}

		scanned++;
	}

	return -ENOSPC;
}

static void mk_ipi_slot_release(struct mk_ipi_data *slot)
{
	WRITE_ONCE(slot->data_size, 0);
	atomic_set_release(&slot->state, MK_IPI_SLOT_EMPTY);
}

/**
 * mk_ipi_ring_drop_pending - Discard everything queued in this kernel's ring
 *
 * Called after the previous instance has halted and immediately before this
 * kernel is re-spawned, when no producers can still access the ring.
 */
void mk_ipi_ring_drop_pending(void)
{
	struct mk_ipi_ring *ring;
	unsigned int i;

	if (!root_instance || !root_instance->ipi_data)
		return;

	ring = &root_instance->ipi_data->ring;
	for (i = 0; i < MK_IPI_RING_SIZE; i++) {
		WRITE_ONCE(ring->entries[i].data_size, 0);
		atomic_set(&ring->entries[i].state, MK_IPI_SLOT_EMPTY);
	}

	atomic_set(&ring->head, 0);
	atomic_set(&ring->tail, 0);
}

/**
 * multikernel_register_handler - Register a callback for multikernel IPI
 * @callback: Function to call when IPI is received
 * @ctx: Context pointer passed to the callback
 * @ipi_type: IPI type this handler should process
 *
 * Returns pointer to handler on success, NULL on failure
 */
struct mk_ipi_handler *multikernel_register_handler(mk_ipi_callback_t callback, void *ctx, unsigned int ipi_type)
{
	struct mk_ipi_handler *handler;
	unsigned long flags;

	if (!callback)
		return NULL;

	handler = kzalloc(sizeof(*handler), GFP_KERNEL);
	if (!handler)
		return NULL;

	handler->callback = callback;
	handler->context = ctx;
	handler->ipi_type = ipi_type;

	raw_spin_lock_irqsave(&mk_handlers_lock, flags);
	handler->next = mk_handlers;
	mk_handlers = handler;
	raw_spin_unlock_irqrestore(&mk_handlers_lock, flags);

	return handler;
}
EXPORT_SYMBOL(multikernel_register_handler);

/**
 * multikernel_unregister_handler - Unregister a multikernel IPI callback
 * @handler: Handler pointer returned from multikernel_register_handler
 */
void multikernel_unregister_handler(struct mk_ipi_handler *handler)
{
	struct mk_ipi_handler **pp, *p;
	unsigned long flags;

	if (!handler)
		return;

	raw_spin_lock_irqsave(&mk_handlers_lock, flags);
	pp = &mk_handlers;
	while ((p = *pp) != NULL) {
		if (p == handler) {
			*pp = p->next;
			break;
		}
		pp = &p->next;
	}
	raw_spin_unlock_irqrestore(&mk_handlers_lock, flags);

	kfree(p);
}
EXPORT_SYMBOL(multikernel_unregister_handler);

/**
 * multikernel_send_ipi_data - Send data to another CPU via IPI
 * @instance_id: Target multikernel instance ID
 * @data: Pointer to data to send
 * @data_size: Size of data
 * @type: User-defined type identifier
 *
 * This function enqueues data into the target instance's IPI ring buffer
 * and sends an IPI to notify the target CPU.
 *
 * Returns 0 on success, negative error code on failure
 */
int multikernel_send_ipi_data(int instance_id, void *data, size_t data_size, unsigned long type)
{
	struct mk_ipi_data *slot;
	struct mk_instance *instance = mk_instance_find(instance_id);
	struct mk_ipi_ring *ring;
	mk_phys_cpu_t target;
	int ret;

	if (!instance)
		return -EINVAL;
	if (data_size > MK_MAX_DATA_SIZE) {
		mk_instance_put(instance);
		return -EINVAL;
	}

	target = mk_cpu_set_first(instance->cpus);
	if (target == MK_PHYS_CPU_INVALID) {
		pr_err("Instance %d has no CPUs to receive the IPI\n", instance_id);
		mk_instance_put(instance);
		return -ENODEV;
	}

	if (!instance->ipi_data) {
		struct mk_shared_data *ipi_data = NULL;

		if (instance->kimage && instance->kimage->mk_ipi)
			ipi_data = phys_to_virt(instance->kimage->mk_ipi);

		if (ipi_data && cmpxchg(&instance->ipi_data, NULL, ipi_data) == NULL) {
			pr_info("Initialized IPI ring buffer for instance %d: phys=0x%llx, virt=%px\n",
				instance->id, (unsigned long long)instance->kimage->mk_ipi,
				ipi_data);
		}
	}

	if (!instance->ipi_data) {
		pr_err("Multikernel IPI buffer not available for instance %d\n", instance_id);
		mk_instance_put(instance);
		return -ENODEV;
	}

	ring = &instance->ipi_data->ring;
	ret = mk_ipi_ring_claim_slot(ring, &slot);
	if (ret) {
		/*
		 * A doorbell can be coalesced while the target is draining this
		 * ring.  Kick it again before reporting backpressure so READY
		 * entries cannot remain stranded without another notification.
		 */
		mk_arch_send_ipi(target);
		printk_deferred(KERN_WARNING
				"multikernel: IPI ring full for instance %d\n",
				instance_id);
		mk_instance_put(instance);
		return ret;
	}

	WRITE_ONCE(slot->data_size, 0);
	slot->sender_cpu = arch_cpu_physical_id(smp_processor_id());
	slot->type = type;

	if (data && data_size > 0)
		memcpy(slot->buffer, data, data_size);

	WRITE_ONCE(slot->data_size, data_size);
	atomic_set_release(&slot->state, MK_IPI_SLOT_READY);
	mk_arch_send_ipi(target);

	mk_instance_put(instance);
	return 0;
}

static void mk_ipi_drain_ring(void)
{
	struct mk_ipi_data *slot;
	struct mk_ipi_handler *handler;
	struct mk_ipi_ring *ring;
	unsigned int tail, idx, scanned;
	size_t data_size;
	int messages_processed = 0;

	if (!root_instance || !root_instance->ipi_data)
		return;

	ring = &root_instance->ipi_data->ring;
	tail = atomic_read(&ring->tail);

	for (scanned = 0; scanned < MK_IPI_RING_SIZE; scanned++) {
		idx = (tail + scanned) & (MK_IPI_RING_SIZE - 1);
		slot = &ring->entries[idx];

		if (atomic_cmpxchg_acquire(&slot->state, MK_IPI_SLOT_READY,
					   MK_IPI_SLOT_CONSUMING) !=
		    MK_IPI_SLOT_READY)
			continue;

		data_size = READ_ONCE(slot->data_size);
		if (data_size > MK_MAX_DATA_SIZE) {
			pr_warn_once("Multikernel IPI slot %u has bad size %zu\n",
				     idx, data_size);
			goto advance_tail;
		}

		/* Dispatch to registered handler */
		raw_spin_lock(&mk_handlers_lock);
		for (handler = mk_handlers; handler; handler = handler->next) {
			if (handler->ipi_type == slot->type && handler->callback) {
				mk_ipi_callback_t cb = handler->callback;
				void *ctx = handler->context;

				raw_spin_unlock(&mk_handlers_lock);
				cb(slot, ctx);
				goto advance_tail;
			}
		}
		raw_spin_unlock(&mk_handlers_lock);

advance_tail:
		mk_ipi_slot_release(slot);
		atomic_set(&ring->tail, (idx + 1) & (MK_IPI_RING_SIZE - 1));
		messages_processed++;

		if (messages_processed >= MK_IPI_RING_SIZE)
			break;
	}
}

/**
 * multikernel_interrupt_handler - Handle the multikernel IPI
 *
 * This function is called when a multikernel IPI is received.
 * Messages are drained here, in interrupt context.
 */
static void multikernel_interrupt_handler(void)
{
	if (!root_instance || !root_instance->ipi_data)
		return;

	/*
	 * Drain here rather than from irq_work. We are already in interrupt
	 * context and every handler is safe to call from it, and irq_work
	 * brings a failure mode with it: the work is a single static
	 * instance, so if it is ever left pending - its self-IPI lost while
	 * the CPU was bringing its APIC up, say - every later queue attempt
	 * is a no-op and the ring never drains again.
	 */
	mk_ipi_drain_ring();
}

/**
 * Generic multikernel interrupt handler - called by the IPI vector
 *
 * This is the function that gets called by the IPI vector handler.
 */
void generic_multikernel_interrupt(void)
{
	multikernel_interrupt_handler();
}

/**
 * mk_has_pending_shutdown - Check if there's a pending shutdown message
 *
 * Checks the dedicated emergency flag published before force-halt NMIs.
 * This path must not depend on claiming a normal IPI ring slot because a
 * crashed producer can leave slots unavailable.
 *
 * Safe to call from NMI context (no locks, read-only peek).
 *
 * Returns: true if shutdown requested, false otherwise
 */
bool mk_has_pending_shutdown(void)
{
	if (!root_instance || !root_instance->ipi_data)
		return false;

	return atomic_read_acquire(&root_instance->ipi_data->emergency_shutdown);
}
