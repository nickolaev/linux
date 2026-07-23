/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_RISCV_MULTIKERNEL_H
#define _ASM_RISCV_MULTIKERNEL_H

#ifndef __ASSEMBLY__
#include <linux/compiler_types.h>

int mk_register_stop_nmi_handler(void);
void mk_force_stop_cpu(int phys_cpu);
int mk_wait_cpu_stopped(int phys_cpu);
void __noreturn mk_enter_pool_state(void *info);
int mk_riscv_send_ipi(unsigned int phys_cpu);
void mk_set_pool_cpu(int cpu, bool is_pool);

#endif
#endif
