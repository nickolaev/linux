// SPDX-License-Identifier: GPL-2.0-only
/* RISC-V Multikernel hart lifecycle using the SBI HSM extension. */
#include <linux/cpu.h>
#include <linux/errno.h>
#include <linux/iopoll.h>
#include <linux/kexec.h>
#include <linux/multikernel.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <asm/multikernel.h>
#include <asm/sbi.h>
#include <asm/smp.h>

static cpumask_t mk_pool_cpus;

void mk_set_pool_cpu(int cpu, bool is_pool)
{
	if (is_pool)
		cpumask_set_cpu(cpu, &mk_pool_cpus);
	else
		cpumask_clear_cpu(cpu, &mk_pool_cpus);
}

bool cpu_is_multikernel_pool(unsigned int cpu)
{
	return cpumask_test_cpu(cpu, &mk_pool_cpus);
}

static struct sbiret mk_hsm_call(unsigned long fid, unsigned long hartid,
				 unsigned long arg1, unsigned long arg2)
{
	return sbi_ecall(SBI_EXT_HSM, fid, hartid, arg1, arg2, 0, 0, 0);
}

static int mk_hsm_status(unsigned long hartid)
{
	struct sbiret ret;

	ret = mk_hsm_call(SBI_EXT_HSM_HART_STATUS, hartid, 0, 0);
	if (ret.error)
		return sbi_err_map_linux_errno(ret.error);
	return ret.value;
}

int mk_spawn_cpu(struct mk_instance *instance, int cpu,
		 struct mk_spawn_context *ctx)
{
	struct kimage *image = instance->kimage;
	unsigned long hartid = cpuid_to_hartid_map(cpu);
	struct sbiret ret;
	int status;

	(void)ctx;
	if (!image || !image->mk_kernel_entry || !image->arch.fdt_addr)
		return -EINVAL;
	if (sbi_probe_extension(SBI_EXT_HSM) <= 0)
		return -EOPNOTSUPP;

	status = mk_hsm_status(hartid);
	if (status < 0)
		return status;
	if (status != SBI_HSM_STATE_STOPPED)
		return -EBUSY;

	flush_icache_all();
	pr_info("multikernel: starting hart %lu at kernel entry 0x%lx with FDT 0x%lx\n",
		hartid, image->mk_kernel_entry, image->arch.fdt_addr);
	ret = mk_hsm_call(SBI_EXT_HSM_HART_START, hartid,
			  image->mk_kernel_entry,
			  image->arch.fdt_addr);
	return ret.error ? sbi_err_map_linux_errno(ret.error) : 0;
}

int mk_setup_host_park(void)
{
	if (sbi_probe_extension(SBI_EXT_HSM) <= 0 ||
	    sbi_probe_extension(SBI_EXT_IPI) <= 0)
		return -EOPNOTSUPP;

	return 0;
}

int mk_repark_instance_to_host(struct mk_instance *instance)
{
	return 0;
}

void mk_free_identity_pgtable(struct mk_ident_pgtable *pgt)
{
}

int mk_register_stop_nmi_handler(void)
{
	return -EOPNOTSUPP;
}

void mk_force_stop_cpu(int phys_cpu)
{
	pr_warn("multikernel: SBI HSM cannot force-stop remote hart %d\n",
		phys_cpu);
}

int mk_wait_cpu_stopped(int phys_cpu)
{
	int status;
	int ret;

	ret = read_poll_timeout(mk_hsm_status, status,
				status < 0 || status == SBI_HSM_STATE_STOPPED,
				1000, USEC_PER_SEC, false, phys_cpu);
	if (ret)
		return ret;

	return status < 0 ? status : 0;
}

void __noreturn mk_enter_pool_state(void *info)
{
	struct sbiret ret;

	local_irq_disable();
	ret = sbi_ecall(SBI_EXT_HSM, SBI_EXT_HSM_HART_STOP,
			0, 0, 0, 0, 0, 0);
	pr_emerg("multikernel: SBI hart stop returned error %ld\n", ret.error);
	for (;;)
		cpu_relax();
}

int mk_riscv_send_ipi(unsigned int phys_cpu)
{
	struct sbiret ret;

	ret = sbi_ecall(SBI_EXT_IPI, SBI_EXT_IPI_SEND_IPI,
			1UL, phys_cpu, 0, 0, 0, 0);
	if (ret.error)
		pr_err("multikernel: failed to send IPI to hart %u: %d\n",
		       phys_cpu, sbi_err_map_linux_errno(ret.error));

	return ret.error ? sbi_err_map_linux_errno(ret.error) : 0;
}
