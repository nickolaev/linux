// SPDX-License-Identifier: GPL-2.0-only
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/kexec.h>
#include <linux/module.h>
#include <linux/multikernel.h>
#include <linux/rculist.h>
#include <linux/smp.h>
#include <linux/wait.h>
#include "internal.h"

static struct mk_ipi_handler *mk_handlers;
static DEFINE_RAW_SPINLOCK(mk_handlers_lock);
static LIST_HEAD(mk_ipi_endpoints);
static DEFINE_RAW_SPINLOCK(mk_ipi_endpoints_lock);
static bool mk_handlers_ready;
static DECLARE_WAIT_QUEUE_HEAD(mk_reply_waitq);

#define MK_REPLY_STATE_MASK	(BIT(MK_REPLY_STATE_BITS) - 1)
#define MK_REPLY_GENERATION_MAX	(U64_MAX >> MK_REPLY_STATE_BITS)

static u64 mk_reply_token(u64 generation, enum mk_reply_state state)
{
	return generation << MK_REPLY_STATE_BITS | state;
}

static u64 mk_reply_generation(u64 token)
{
	return token >> MK_REPLY_STATE_BITS;
}

static enum mk_reply_state mk_reply_state(u64 token)
{
	return token & MK_REPLY_STATE_MASK;
}

int mk_reply_reserve(struct mk_shared_data *shared, u32 kind, u64 request_id,
		     struct mk_reply_handle *reply)
{
	struct mk_reply_table *table;
	unsigned int i;

	if (!shared || !reply || !request_id || !kind)
		return -EINVAL;

	table = &shared->replies;
	for (i = 0; i < MK_REPLY_SLOTS; i++) {
		struct mk_reply_slot *slot = &table->slots[i];
		u64 generation;
		u64 claim;
		u64 old;

		old = atomic64_read(&slot->state_generation);
		if (mk_reply_state(old) != MK_REPLY_FREE)
			continue;
		generation = mk_reply_generation(old) + 1;
		if (!generation || generation > MK_REPLY_GENERATION_MAX)
			generation = 1;
		claim = mk_reply_token(generation, MK_REPLY_WRITING);
		if (atomic64_cmpxchg_acquire(&slot->state_generation, old,
					     claim) != old)
			continue;

		WRITE_ONCE(slot->request_id, request_id);
		WRITE_ONCE(slot->kind, kind);
		WRITE_ONCE(slot->status, -ETIMEDOUT);
		WRITE_ONCE(slot->value, ~0U);
		WRITE_ONCE(slot->owner_cpu, MK_REPLY_OWNER_INVALID);
		atomic64_set_release(&slot->state_generation,
				     mk_reply_token(generation,
						    MK_REPLY_RESERVED));
		reply->slot = i;
		reply->kind = kind;
		reply->request_id = request_id;
		reply->generation = generation;
		return 0;
	}

	atomic_inc(&table->occupied_failures);
	return -ENOSPC;
}

static int mk_reply_take_ready(struct mk_shared_data *shared,
			       struct mk_reply_handle *reply,
			       s32 *status, u32 *value)
{
	struct mk_reply_slot *slot = &shared->replies.slots[reply->slot];
	u64 ready = mk_reply_token(reply->generation, MK_REPLY_READY);
	u64 free = mk_reply_token(reply->generation, MK_REPLY_FREE);

	if (atomic64_read_acquire(&slot->state_generation) != ready)
		return -EAGAIN;
	if (READ_ONCE(slot->request_id) != reply->request_id ||
	    READ_ONCE(slot->kind) != reply->kind)
		return -EPROTO;
	if (status)
		*status = READ_ONCE(slot->status);
	if (value)
		*value = READ_ONCE(slot->value);
	WRITE_ONCE(slot->owner_cpu, MK_REPLY_OWNER_INVALID);
	if (atomic64_cmpxchg_release(&slot->state_generation, ready, free) !=
	    ready)
		return -EAGAIN;
	return 0;
}

static int mk_reply_cancel(struct mk_shared_data *shared,
			   struct mk_reply_handle *reply, bool atomic_timeout)
{
	struct mk_reply_table *table = &shared->replies;
	struct mk_reply_slot *slot = &table->slots[reply->slot];
	u64 reserved = mk_reply_token(reply->generation, MK_REPLY_RESERVED);
	u64 writing = mk_reply_token(reply->generation, MK_REPLY_WRITING);
	u64 executing = mk_reply_token(reply->generation, MK_REPLY_EXECUTING);
	u64 committed = mk_reply_token(reply->generation, MK_REPLY_COMMITTED);
	u64 abandoned = mk_reply_token(reply->generation, MK_REPLY_ABANDONED);
	u64 ready = mk_reply_token(reply->generation, MK_REPLY_READY);
	u64 free = mk_reply_token(reply->generation, MK_REPLY_FREE);
	u64 token;

	if (atomic64_cmpxchg_release(&slot->state_generation, reserved, free) ==
	    reserved)
		goto cancelled;

	for (;;) {
		token = atomic64_read_acquire(&slot->state_generation);
		if (token == ready)
			return 1;
		if (token == committed)
			return -EINPROGRESS;
		if (token == writing &&
		    atomic64_cmpxchg_release(&slot->state_generation, writing,
					     abandoned) == writing)
			break;
		if (token == executing &&
		    atomic64_cmpxchg_release(&slot->state_generation, executing,
					     committed) == executing) {
			atomic_inc(&table->indeterminate_timeouts);
			if (atomic_timeout)
				atomic_inc(&table->atomic_timeouts);
			return -EINPROGRESS;
		}
		if (token != writing && token != executing)
			break;
	}

cancelled:
	atomic_inc(&table->cancelled_slots);
	if (atomic_timeout)
		atomic_inc(&table->atomic_timeouts);
	return 0;
}

static bool mk_reply_wait_done(struct mk_shared_data *shared,
			       const struct mk_reply_handle *reply)
{
	struct mk_reply_slot *slot = &shared->replies.slots[reply->slot];
	u64 token;

	token = atomic64_read_acquire(&slot->state_generation);
	return token == mk_reply_token(reply->generation, MK_REPLY_READY) ||
	       mk_reply_generation(token) != reply->generation ||
	       mk_reply_state(token) == MK_REPLY_FREE;
}

int mk_reply_wait_atomic(struct mk_shared_data *shared,
			 struct mk_reply_handle *reply, unsigned int timeout_us,
			 s32 *status, u32 *value)
{
	u64 deadline;
	int cancelled;

	if (!shared || !reply || reply->slot >= MK_REPLY_SLOTS)
		return -EINVAL;
	deadline = ktime_get_mono_fast_ns() + (u64)timeout_us * NSEC_PER_USEC;
	for (;;) {
		int ret = mk_reply_take_ready(shared, reply, status, value);

		if (!ret)
			return 0;
		if (ret != -EAGAIN)
			return ret;
		if (ktime_get_mono_fast_ns() >= deadline)
			break;
		cpu_relax();
	}

	cancelled = mk_reply_cancel(shared, reply, true);
	if (cancelled > 0)
		return mk_reply_take_ready(shared, reply, status, value);
	if (cancelled < 0)
		return cancelled;
	return -ETIMEDOUT;
}

int mk_reply_wait(struct mk_shared_data *shared,
		  struct mk_reply_handle *reply, unsigned int timeout_ms,
		  s32 *status, u32 *value)
{
	long waited;
	int cancelled;
	int ret;

	if (!shared || !reply || reply->slot >= MK_REPLY_SLOTS)
		return -EINVAL;
	waited = wait_event_timeout(mk_reply_waitq,
				    mk_reply_wait_done(shared, reply),
				    msecs_to_jiffies(timeout_ms));
	if (!waited) {
		cancelled = mk_reply_cancel(shared, reply, false);
		if (cancelled > 0)
			return mk_reply_take_ready(shared, reply, status, value);
		if (cancelled < 0)
			return cancelled;
		return -ETIMEDOUT;
	}
	ret = mk_reply_take_ready(shared, reply, status, value);
	if (ret == -EAGAIN)
		ret = -ESTALE;
	return ret;
}

void mk_reply_release(struct mk_shared_data *shared,
		      struct mk_reply_handle *reply)
{
	if (!shared || !reply || reply->slot >= MK_REPLY_SLOTS)
		return;
	mk_reply_cancel(shared, reply, false);
}

int mk_reply_claim(struct mk_instance *instance,
		   const struct mk_reply_handle *reply)
{
	struct mk_shared_data *shared;
	struct mk_reply_slot *slot;
	u64 writing;
	u64 abandoned;
	u64 reserved;
	u64 free;
	mk_phys_cpu_t owner;

	if (!instance || !reply || reply->slot >= MK_REPLY_SLOTS)
		return -EINVAL;
	shared = instance->ipi_data;
	if (!shared)
		return -ENODEV;

	slot = &shared->replies.slots[reply->slot];
	reserved = mk_reply_token(reply->generation, MK_REPLY_RESERVED);
	writing = mk_reply_token(reply->generation, MK_REPLY_WRITING);
	abandoned = mk_reply_token(reply->generation, MK_REPLY_ABANDONED);
	free = mk_reply_token(reply->generation, MK_REPLY_FREE);
	owner = arch_cpu_physical_id(smp_processor_id());
	if (cmpxchg(&slot->owner_cpu, MK_REPLY_OWNER_INVALID, owner) !=
	    MK_REPLY_OWNER_INVALID) {
		atomic_inc(&shared->replies.late_replies);
		return -ESTALE;
	}
	if (atomic64_cmpxchg_acquire(&slot->state_generation, reserved,
				     writing) != reserved) {
		cmpxchg(&slot->owner_cpu, owner, MK_REPLY_OWNER_INVALID);
		atomic_inc(&shared->replies.late_replies);
		return -ESTALE;
	}
	if (READ_ONCE(slot->request_id) != reply->request_id ||
	    READ_ONCE(slot->kind) != reply->kind) {
		u64 old;

		WRITE_ONCE(slot->owner_cpu, MK_REPLY_OWNER_INVALID);
		old = atomic64_cmpxchg_release(&slot->state_generation, writing,
					       free);
		if (old == abandoned)
			old = atomic64_cmpxchg_release(&slot->state_generation,
						       abandoned, free);
		if (old == writing || old == abandoned)
			wake_up_all(&mk_reply_waitq);
		atomic_inc(&shared->replies.late_replies);
		return -ESTALE;
	}
	return 0;
}

int mk_reply_begin_execute(struct mk_instance *instance,
			   const struct mk_reply_handle *reply)
{
	struct mk_reply_slot *slot;
	u64 writing;
	u64 executing;
	u64 old;

	if (!instance || !instance->ipi_data || !reply ||
	    reply->slot >= MK_REPLY_SLOTS)
		return -EINVAL;
	slot = &instance->ipi_data->replies.slots[reply->slot];
	writing = mk_reply_token(reply->generation, MK_REPLY_WRITING);
	executing = mk_reply_token(reply->generation, MK_REPLY_EXECUTING);
	old = atomic64_cmpxchg_acquire(&slot->state_generation, writing,
				       executing);
	if (old == writing)
		return 0;
	if (old == mk_reply_token(reply->generation, MK_REPLY_ABANDONED))
		return -ECANCELED;
	return -ESTALE;
}

int mk_reply_publish(struct mk_instance *instance,
		     const struct mk_reply_handle *reply, s32 status, u32 value)
{
	struct mk_shared_data *shared;
	struct mk_reply_slot *slot;
	mk_phys_cpu_t target;
	u64 writing;
	u64 executing;
	u64 committed;
	u64 abandoned;
	u64 ready;
	u64 free;
	u64 old;

	if (!instance || !reply || reply->slot >= MK_REPLY_SLOTS)
		return -EINVAL;
	shared = instance->ipi_data;
	if (!shared)
		return -ENODEV;
	slot = &shared->replies.slots[reply->slot];
	writing = mk_reply_token(reply->generation, MK_REPLY_WRITING);
	executing = mk_reply_token(reply->generation, MK_REPLY_EXECUTING);
	committed = mk_reply_token(reply->generation, MK_REPLY_COMMITTED);
	abandoned = mk_reply_token(reply->generation, MK_REPLY_ABANDONED);
	ready = mk_reply_token(reply->generation, MK_REPLY_READY);
	free = mk_reply_token(reply->generation, MK_REPLY_FREE);

	WRITE_ONCE(slot->status, status);
	WRITE_ONCE(slot->value, value);
	old = atomic64_cmpxchg_release(&slot->state_generation, executing, ready);
	if (old == writing)
		old = atomic64_cmpxchg_release(&slot->state_generation, writing,
					       ready);
	if (old == abandoned) {
		WRITE_ONCE(slot->owner_cpu, MK_REPLY_OWNER_INVALID);
		atomic64_set_release(&slot->state_generation, free);
		atomic_inc(&shared->replies.late_replies);
		return -ESTALE;
	}
	if (old == committed) {
		WRITE_ONCE(slot->owner_cpu, MK_REPLY_OWNER_INVALID);
		atomic64_set_release(&slot->state_generation, free);
		atomic_inc(&shared->replies.late_replies);
		return -ESTALE;
	}
	if (old != executing && old != writing) {
		atomic_inc(&shared->replies.late_replies);
		return -EIO;
	}

	target = mk_instance_irq_route_load(instance);
	if (target == MK_PHYS_CPU_INVALID)
		return -ENODEV;
	mk_arch_send_ipi(target);
	return 0;
}

void mk_reply_scan(struct mk_shared_data *shared)
{
	unsigned int i;

	if (!shared)
		return;
	for (i = 0; i < MK_REPLY_SLOTS; i++) {
		struct mk_reply_slot *slot = &shared->replies.slots[i];
		u64 token = atomic64_read_acquire(&slot->state_generation);

		if (mk_reply_state(token) == MK_REPLY_READY) {
			wake_up_all(&mk_reply_waitq);
			return;
		}
	}
}

void mk_reply_recover_halted(struct mk_shared_data *shared,
			     const struct mk_cpu_set *halted_cpus)
{
	struct mk_reply_table *table;
	unsigned int i;

	if (!shared || !halted_cpus)
		return;
	table = &shared->replies;
	for (i = 0; i < MK_REPLY_SLOTS; i++) {
		struct mk_reply_slot *slot = &table->slots[i];
		enum mk_reply_state state;
		mk_phys_cpu_t owner;
		u64 token, free;

		token = atomic64_read_acquire(&slot->state_generation);
		state = mk_reply_state(token);
		if (state != MK_REPLY_RESERVED && state != MK_REPLY_WRITING &&
		    state != MK_REPLY_EXECUTING && state != MK_REPLY_COMMITTED &&
		    state != MK_REPLY_ABANDONED)
			continue;

		owner = READ_ONCE(slot->owner_cpu);
		if (owner == MK_REPLY_OWNER_INVALID ||
		    !mk_cpu_set_contains(halted_cpus, owner))
			continue;

		free = mk_reply_token(mk_reply_generation(token), MK_REPLY_FREE);
		if (atomic64_cmpxchg_release(&slot->state_generation, token, free) !=
		    token)
			continue;

		WRITE_ONCE(slot->owner_cpu, MK_REPLY_OWNER_INVALID);
		atomic_inc(&table->cancelled_slots);
		wake_up_all(&mk_reply_waitq);
	}
}

static struct mk_shared_data *mk_instance_ipi_area(struct mk_instance *instance)
{
	struct mk_shared_data *ipi_data;

	if (instance->ipi_data)
		return instance->ipi_data;
	if (!instance->kimage || !instance->kimage->mk_ipi)
		return NULL;
	ipi_data = phys_to_virt(instance->kimage->mk_ipi);
	cmpxchg(&instance->ipi_data, NULL, ipi_data);
	return instance->ipi_data;
}

int mk_ipi_endpoint_init(struct mk_instance *instance, bool parent_side)
{
	struct mk_ipi_endpoint *endpoint = &instance->ipi_endpoint;
	unsigned long flags;

	if (!mk_instance_ipi_area(instance))
		return -ENODEV;
	if (endpoint->registered)
		return 0;
	raw_spin_lock_init(&endpoint->tx_lock);
	raw_spin_lock_init(&endpoint->rx_lock);
	endpoint->tx = parent_side ? &instance->ipi_data->to_child :
				     &instance->ipi_data->to_parent;
	endpoint->rx = parent_side ? &instance->ipi_data->to_parent :
				     &instance->ipi_data->to_child;
	endpoint->tx_head = 0;
	endpoint->rx_tail = 0;
	endpoint->tx_enabled = true;
	endpoint->rx_dispatching = false;
	endpoint->parent_side = parent_side;
	INIT_LIST_HEAD(&endpoint->rx_node);
	raw_spin_lock_irqsave(&mk_ipi_endpoints_lock, flags);
	list_add_tail_rcu(&endpoint->rx_node, &mk_ipi_endpoints);
	endpoint->registered = true;
	raw_spin_unlock_irqrestore(&mk_ipi_endpoints_lock, flags);
	return 0;
}

void mk_ipi_endpoint_close(struct mk_instance *instance)
{
	struct mk_ipi_endpoint *endpoint = &instance->ipi_endpoint;
	unsigned long flags;

	if (!endpoint->registered)
		return;
	raw_spin_lock_irqsave(&endpoint->tx_lock, flags);
	endpoint->tx_enabled = false;
	raw_spin_unlock_irqrestore(&endpoint->tx_lock, flags);
}

void mk_ipi_endpoint_unregister(struct mk_instance *instance)
{
	struct mk_ipi_endpoint *endpoint = &instance->ipi_endpoint;
	unsigned long flags;

	if (!endpoint->registered)
		return;
	mk_ipi_endpoint_close(instance);
	raw_spin_lock_irqsave(&mk_ipi_endpoints_lock, flags);
	if (endpoint->registered) {
		list_del_rcu(&endpoint->rx_node);
		endpoint->registered = false;
	}
	raw_spin_unlock_irqrestore(&mk_ipi_endpoints_lock, flags);
	synchronize_rcu();
}

void mk_ipi_link_reset(struct mk_instance *instance, int parent_id,
		       int child_id, mk_phys_cpu_t parent_cpu,
		       mk_phys_cpu_t child_cpu)
{
	struct mk_shared_data *shared = mk_instance_ipi_area(instance);
	u64 epoch;

	if (!shared)
		return;
	epoch = READ_ONCE(shared->spawn_epoch) + 1;
	if (!epoch)
		epoch = 1;
	mk_ipi_endpoint_unregister(instance);
	mk_shared_data_reset(shared);
	WRITE_ONCE(shared->parent_id, parent_id);
	WRITE_ONCE(shared->child_id, child_id);
	WRITE_ONCE(shared->parent_doorbell_cpu, parent_cpu);
	WRITE_ONCE(shared->child_doorbell_cpu, child_cpu);
	WRITE_ONCE(shared->spawn_epoch, epoch);
	mk_ipi_endpoint_init(instance, true);
}

struct mk_ipi_handler *
multikernel_register_handler(mk_ipi_callback_t callback, void *ctx,
			     unsigned int ipi_type)
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

void multikernel_unregister_handler(struct mk_ipi_handler *handler)
{
	struct mk_ipi_handler **pp, *p = NULL;
	unsigned long flags;

	if (!handler)
		return;
	raw_spin_lock_irqsave(&mk_handlers_lock, flags);
	for (pp = &mk_handlers; (p = *pp); pp = &p->next) {
		if (p == handler) {
			*pp = p->next;
			break;
		}
	}
	raw_spin_unlock_irqrestore(&mk_handlers_lock, flags);
	kfree(p);
}
EXPORT_SYMBOL(multikernel_unregister_handler);

int mk_arm_force_halt(struct mk_instance *instance)
{
	struct mk_shared_data *shared = mk_instance_ipi_area(instance);

	if (!shared)
		return -ENODEV;
	WRITE_ONCE(shared->force_halt, 1);
	smp_wmb();
	return 0;
}

static int __mk_send_ipi_data(struct mk_instance *instance,
			      mk_phys_cpu_t target, void *data,
			      size_t data_size, unsigned long type)
{
	struct mk_ipi_endpoint *endpoint;
	struct mk_ipi_data *slot;
	unsigned long flags;
	u32 idx;
	int ret = 0;

	if (!instance || target == MK_PHYS_CPU_INVALID ||
	    data_size > MK_MAX_DATA_SIZE || (data_size && !data))
		return -EINVAL;
	endpoint = &instance->ipi_endpoint;
	if (!READ_ONCE(endpoint->registered))
		return -ESHUTDOWN;
	if (endpoint->parent_side)
		WRITE_ONCE(instance->ipi_data->child_doorbell_cpu, target);
	raw_spin_lock_irqsave(&endpoint->tx_lock, flags);
	if (!endpoint->tx_enabled) {
		ret = -ESHUTDOWN;
		goto unlock;
	}
	idx = endpoint->tx_head & (MK_IPI_RING_SIZE - 1);
	slot = &endpoint->tx->entries[idx];
	/* Pair with the receiver's release when it makes the slot reusable. */
	if (smp_load_acquire(&slot->ready)) {
		ret = -ENOSPC;
		goto unlock;
	}
	WRITE_ONCE(slot->sender_cpu, arch_cpu_physical_id(smp_processor_id()));
	WRITE_ONCE(slot->type, type);
	WRITE_ONCE(slot->data_size, data_size);
	if (data_size)
		memcpy(slot->buffer, data, data_size);
	/* Publish all message fields before the receiver observes readiness. */
	smp_store_release(&slot->ready, 1);
	endpoint->tx_head++;
unlock:
	raw_spin_unlock_irqrestore(&endpoint->tx_lock, flags);
	if (!ret)
		mk_arch_send_ipi(target);
	return ret;
}

int mk_send_ipi_data_to_cpu(struct mk_instance *instance,
			    mk_phys_cpu_t target, void *data,
			    size_t data_size, unsigned long type)
{
	return __mk_send_ipi_data(instance, target, data, data_size, type);
}

int mk_send_ipi_data(struct mk_instance *instance, void *data,
		     size_t data_size, unsigned long type)
{
	struct mk_ipi_endpoint *endpoint;
	mk_phys_cpu_t target;
	int ret;

	if (!instance)
		return -EINVAL;
	endpoint = &instance->ipi_endpoint;
	if (!endpoint->registered) {
		ret = mk_ipi_endpoint_init(instance, true);
		if (ret)
			return ret;
	}
	target = endpoint->parent_side ? mk_cpu_set_first(instance->cpus) :
		 READ_ONCE(instance->ipi_data->parent_doorbell_cpu);
	if (target == MK_PHYS_CPU_INVALID) {
		pr_err("Instance %d has no CPUs to receive the IPI\n",
		       instance->id);
		return -ENODEV;
	}
	return __mk_send_ipi_data(instance, target, data, data_size, type);
}

int multikernel_send_ipi_data(int instance_id, void *data, size_t data_size,
			      unsigned long type)
{
	struct mk_instance *instance = mk_instance_find(instance_id);
	int ret;

	if (!instance)
		return -EINVAL;
	ret = mk_send_ipi_data(instance, data, data_size, type);
	mk_instance_put(instance);
	return ret;
}

static void mk_ipi_dispatch(struct mk_ipi_data *slot)
{
	struct mk_ipi_handler *handler;
	mk_ipi_callback_t callback = NULL;
	void *context = NULL;
	unsigned long flags;

	if (READ_ONCE(slot->data_size) > MK_MAX_DATA_SIZE)
		return;
	raw_spin_lock_irqsave(&mk_handlers_lock, flags);
	for (handler = mk_handlers; handler; handler = handler->next) {
		if (handler->ipi_type == READ_ONCE(slot->type)) {
			callback = handler->callback;
			context = handler->context;
			break;
		}
	}
	raw_spin_unlock_irqrestore(&mk_handlers_lock, flags);
	if (callback)
		callback(slot, context);
}

static void mk_ipi_drain_endpoint(struct mk_ipi_endpoint *endpoint)
{
	struct mk_ipi_data *slot;
	unsigned long flags;
	u32 idx;

	raw_spin_lock_irqsave(&endpoint->rx_lock, flags);
	if (endpoint->rx_dispatching) {
		raw_spin_unlock_irqrestore(&endpoint->rx_lock, flags);
		return;
	}
	endpoint->rx_dispatching = true;
	raw_spin_unlock_irqrestore(&endpoint->rx_lock, flags);
	for (;;) {
		idx = endpoint->rx_tail & (MK_IPI_RING_SIZE - 1);
		slot = &endpoint->rx->entries[idx];
		/* Pair with the producer's release publication of this slot. */
		if (!smp_load_acquire(&slot->ready)) {
			raw_spin_lock_irqsave(&endpoint->rx_lock, flags);
			endpoint->rx_dispatching = false;
			/* Close the empty-ring handoff race with a new publication. */
			if (smp_load_acquire(&slot->ready)) {
				endpoint->rx_dispatching = true;
				raw_spin_unlock_irqrestore(&endpoint->rx_lock, flags);
				continue;
			}
			raw_spin_unlock_irqrestore(&endpoint->rx_lock, flags);
			return;
		}
		mk_ipi_dispatch(slot);
		/* The callback must finish reading before the slot is reusable. */
		smp_store_release(&slot->ready, 0);
		endpoint->rx_tail++;
	}
}

static void mk_ipi_drain_all(void)
{
	struct mk_ipi_endpoint *endpoint;

	if (root_instance && root_instance->ipi_data) {
		mk_reply_scan(root_instance->ipi_data);
		mk_pci_irq_mailbox_drain(root_instance->ipi_data);
	}
	if (!READ_ONCE(mk_handlers_ready))
		return;
	rcu_read_lock();
	list_for_each_entry_rcu(endpoint, &mk_ipi_endpoints, rx_node)
		mk_ipi_drain_endpoint(endpoint);
	rcu_read_unlock();
}

void mk_ipi_handlers_enable(void)
{
	WRITE_ONCE(mk_handlers_ready, true);
	mk_ipi_drain_all();
}

void mk_poll_ipi_messages(void)
{
	unsigned long flags;

	local_irq_save(flags);
	mk_ipi_drain_all();
	local_irq_restore(flags);
}

void generic_multikernel_interrupt(void)
{
	mk_ipi_drain_all();
}

bool mk_has_pending_shutdown(void)
{
	return root_instance && root_instance->ipi_data &&
		READ_ONCE(root_instance->ipi_data->force_halt);
}
