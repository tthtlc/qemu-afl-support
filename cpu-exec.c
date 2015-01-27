/*
 *  emulator main execution loop
 *
 *  Copyright (c) 2003-2005 Fabrice Bellard
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#include "config.h"
#include "cpu.h"
#include "trace.h"
#include "disas/disas.h"
#include "tcg.h"
#include "qemu/atomic.h"
#include "sysemu/qtest.h"
#include "qemu/timer.h"

/* -icount align implementation. */

typedef struct SyncClocks {
    int64_t diff_clk;
    int64_t last_cpu_icount;
    int64_t realtime_clock;
} SyncClocks;

#if !defined(CONFIG_USER_ONLY)
/* Allow the guest to have a max 3ms advance.
 * The difference between the 2 clocks could therefore
 * oscillate around 0.
 */
#define VM_CLOCK_ADVANCE 3000000
#define THRESHOLD_REDUCE 1.5
#define MAX_DELAY_PRINT_RATE 2000000000LL
#define MAX_NB_PRINTS 100

static void align_clocks(SyncClocks *sc, const CPUState *cpu)
{
    int64_t cpu_icount;

    if (!icount_align_option) {
        return;
    }

    cpu_icount = cpu->icount_extra + cpu->icount_decr.u16.low;
    sc->diff_clk += cpu_icount_to_ns(sc->last_cpu_icount - cpu_icount);
    sc->last_cpu_icount = cpu_icount;

    if (sc->diff_clk > VM_CLOCK_ADVANCE) {
#ifndef _WIN32
        struct timespec sleep_delay, rem_delay;
        sleep_delay.tv_sec = sc->diff_clk / 1000000000LL;
        sleep_delay.tv_nsec = sc->diff_clk % 1000000000LL;
        if (nanosleep(&sleep_delay, &rem_delay) < 0) {
            sc->diff_clk -= (sleep_delay.tv_sec - rem_delay.tv_sec) * 1000000000LL;
            sc->diff_clk -= sleep_delay.tv_nsec - rem_delay.tv_nsec;
        } else {
            sc->diff_clk = 0;
        }
#else
        Sleep(sc->diff_clk / SCALE_MS);
        sc->diff_clk = 0;
#endif
    }
}

static void print_delay(const SyncClocks *sc)
{
    static float threshold_delay;
    static int64_t last_realtime_clock;
    static int nb_prints;

    if (icount_align_option &&
        sc->realtime_clock - last_realtime_clock >= MAX_DELAY_PRINT_RATE &&
        nb_prints < MAX_NB_PRINTS) {
        if ((-sc->diff_clk / (float)1000000000LL > threshold_delay) ||
            (-sc->diff_clk / (float)1000000000LL <
             (threshold_delay - THRESHOLD_REDUCE))) {
            threshold_delay = (-sc->diff_clk / 1000000000LL) + 1;
            printf("Warning: The guest is now late by %.1f to %.1f seconds\n",
                   threshold_delay - 1,
                   threshold_delay);
            nb_prints++;
            last_realtime_clock = sc->realtime_clock;
        }
    }
}

static void init_delay_params(SyncClocks *sc,
                              const CPUState *cpu)
{
    if (!icount_align_option) {
        return;
    }
    sc->realtime_clock = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    sc->diff_clk = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                   sc->realtime_clock +
                   cpu_get_clock_offset();
    sc->last_cpu_icount = cpu->icount_extra + cpu->icount_decr.u16.low;
    if (sc->diff_clk < max_delay) {
        max_delay = sc->diff_clk;
    }
    if (sc->diff_clk > max_advance) {
        max_advance = sc->diff_clk;
    }

    /* Print every 2s max if the guest is late. We limit the number
       of printed messages to NB_PRINT_MAX(currently 100) */
    print_delay(sc);
}
#else
static void align_clocks(SyncClocks *sc, const CPUState *cpu)
{
}

static void init_delay_params(SyncClocks *sc, const CPUState *cpu)
{
}
#endif /* CONFIG USER ONLY */

void cpu_loop_exit(CPUState *cpu)
{
    cpu->current_tb = NULL;
    siglongjmp(cpu->jmp_env, 1);
}

/* exit the current TB from a signal handler. The host registers are
   restored in a state compatible with the CPU emulator
 */
#if defined(CONFIG_SOFTMMU)
void cpu_resume_from_signal(CPUState *cpu, void *puc)
{
    /* XXX: restore cpu registers saved in host registers */

    cpu->exception_index = -1;
    siglongjmp(cpu->jmp_env, 1);
}
#endif

/* Execute a TB, and fix up the CPU state afterwards if necessary */
static inline tcg_target_ulong cpu_tb_exec(CPUState *cpu, uint8_t *tb_ptr)
{
    CPUArchState *env = cpu->env_ptr;
    uintptr_t next_tb;

#if defined(DEBUG_DISAS)
    if (qemu_loglevel_mask(CPU_LOG_TB_CPU)) {
#if defined(TARGET_I386)
        log_cpu_state(cpu, CPU_DUMP_CCOP);
#elif defined(TARGET_M68K)
        /* ??? Should not modify env state for dumping.  */
        cpu_m68k_flush_flags(env, env->cc_op);
        env->cc_op = CC_OP_FLAGS;
        env->sr = (env->sr & 0xffe0) | env->cc_dest | (env->cc_x << 4);
        log_cpu_state(cpu, 0);
#else
        log_cpu_state(cpu, 0);
#endif
    }
#endif /* DEBUG_DISAS */

    cpu->can_do_io = 0;
    next_tb = tcg_qemu_tb_exec(env, tb_ptr);
    cpu->can_do_io = 1;
    trace_exec_tb_exit((void *) (next_tb & ~TB_EXIT_MASK),
                       next_tb & TB_EXIT_MASK);

    if ((next_tb & TB_EXIT_MASK) > TB_EXIT_IDX1) {
        /* We didn't start executing this TB (eg because the instruction
         * counter hit zero); we must restore the guest PC to the address
         * of the start of the TB.
         */
        CPUClass *cc = CPU_GET_CLASS(cpu);
        TranslationBlock *tb = (TranslationBlock *)(next_tb & ~TB_EXIT_MASK);
        if (cc->synchronize_from_tb) {
            cc->synchronize_from_tb(cpu, tb);
        } else {
            assert(cc->set_pc);
            cc->set_pc(cpu, tb->pc);
        }
    }
    if ((next_tb & TB_EXIT_MASK) == TB_EXIT_REQUESTED) {
        /* We were asked to stop executing TBs (probably a pending
         * interrupt. We've now stopped, so clear the flag.
         */
        cpu->tcg_exit_req = 0;
    }
    return next_tb;
}

/* Execute the code without caching the generated code. An interpreter
   could be used if available. */
static void cpu_exec_nocache(CPUArchState *env, int max_cycles,
                             TranslationBlock *orig_tb)
{
    CPUState *cpu = ENV_GET_CPU(env);
    TranslationBlock *tb;
    target_ulong pc = orig_tb->pc;
    target_ulong cs_base = orig_tb->cs_base;
    uint64_t flags = orig_tb->flags;

    /* Should never happen.
       We only end up here when an existing TB is too long.  */
    if (max_cycles > CF_COUNT_MASK)
        max_cycles = CF_COUNT_MASK;

    /* tb_gen_code can flush our orig_tb, invalidate it now */
    tb_phys_invalidate(orig_tb, -1);
    tb = tb_gen_code(cpu, pc, cs_base, flags,
                     max_cycles | CF_NOCACHE);
    cpu->current_tb = tb;
    /* execute the generated code */
    trace_exec_tb_nocache(tb, tb->pc);
    cpu_tb_exec(cpu, tb->tc_ptr);
    cpu->current_tb = NULL;
    tb_phys_invalidate(tb, -1);
    tb_free(tb);
}

void afl_fork_faster_writer(target_ulong pc, target_ulong cs_base, uint64_t flags);


static TranslationBlock *tb_find_slow(CPUArchState *env,
                                      target_ulong pc,
                                      target_ulong cs_base,
                                      uint64_t flags)
{
    CPUState *cpu = ENV_GET_CPU(env);
    TranslationBlock *tb, **ptb1;
    unsigned int h;
    tb_page_addr_t phys_pc, phys_page1;
    target_ulong virt_page2;

    tcg_ctx.tb_ctx.tb_invalidated_flag = 0;

    /* find translated block using physical mappings */
    phys_pc = get_page_addr_code(env, pc);
    phys_page1 = phys_pc & TARGET_PAGE_MASK;
    h = tb_phys_hash_func(phys_pc);
    ptb1 = &tcg_ctx.tb_ctx.tb_phys_hash[h];
    for(;;) {
        tb = *ptb1;
        if (!tb)
            goto not_found;
        if (tb->pc == pc &&
            tb->page_addr[0] == phys_page1 &&
            tb->cs_base == cs_base &&
            tb->flags == flags) {
            /* check next page if needed */
            if (tb->page_addr[1] != -1) {
                tb_page_addr_t phys_page2;

                virt_page2 = (pc & TARGET_PAGE_MASK) +
                    TARGET_PAGE_SIZE;
                phys_page2 = get_page_addr_code(env, virt_page2);
                if (tb->page_addr[1] == phys_page2)
                    goto found;
            } else {
                goto found;
            }
        }
        ptb1 = &tb->phys_hash_next;
    }
 not_found:
   /* if no translated code available, then translate it now */
    afl_fork_faster_writer(pc, cs_base, flags);
    tb = tb_gen_code(cpu, pc, cs_base, flags, 0);

 found:
    /* Move the last found TB to the head of the list */
    if (likely(*ptb1)) {
        *ptb1 = tb->phys_hash_next;
        tb->phys_hash_next = tcg_ctx.tb_ctx.tb_phys_hash[h];
        tcg_ctx.tb_ctx.tb_phys_hash[h] = tb;
    }
    /* we add the TB in the virtual pc hash table */
    cpu->tb_jmp_cache[tb_jmp_cache_hash_func(pc)] = tb;
    return tb;
}

static inline TranslationBlock *tb_find_fast(CPUArchState *env)
{
    CPUState *cpu = ENV_GET_CPU(env);
    TranslationBlock *tb;
    target_ulong cs_base, pc;
    int flags;

    /* we record a subset of the CPU state. It will
       always be the same before a given translated block
       is executed. */
    cpu_get_tb_cpu_state(env, &pc, &cs_base, &flags);
    tb = cpu->tb_jmp_cache[tb_jmp_cache_hash_func(pc)];
    if (unlikely(!tb || tb->pc != pc || tb->cs_base != cs_base ||
                 tb->flags != flags)) {
        tb = tb_find_slow(env, pc, cs_base, flags);
    }
    return tb;
}

static void cpu_handle_debug_exception(CPUArchState *env)
{
    CPUState *cpu = ENV_GET_CPU(env);
    CPUClass *cc = CPU_GET_CLASS(cpu);
    CPUWatchpoint *wp;

    if (!cpu->watchpoint_hit) {
        QTAILQ_FOREACH(wp, &cpu->watchpoints, entry) {
            wp->flags &= ~BP_WATCHPOINT_HIT;
        }
    }

    cc->debug_excp_handler(cpu);
}

/* main execution loop */

volatile sig_atomic_t exit_request;

#define AFL_FORKSERVER_FD 198
#define AFL_FORKFASTER_FD 200
static u_int8_t *__afl_area_ptr;

// must be sync'd with afl config.h ...
#define MAP_SIZE_POW2       16
#define MAP_SIZE            (1 << MAP_SIZE_POW2)

#include <sys/shm.h>

abi_ulong afl_entry_point;		// ELF entry point of the executable (_start)
abi_ulong afl_start_code, afl_end_code;	// ELF .text start and finish
int afl_instrument_all;
int afl_forkserver_running;

static void afl_fork_faster_reader(CPUArchState *env);
static void afl_setup(void);
static void afl_forkserver(CPUArchState *env);
inline void afl_maybe_log(abi_ulong cur_loc);

/*
 * setup the AFL fuzzing support if it's ran under afl-fuzz
 */

static void afl_setup(void)
{
	char *shm_env;
	static int has_ran_before;
	int shm_id;

	if(has_ran_before) {
		qemu_log("afl_setup() - has_ran_before = true, unexpected. threading?\n");
		exit(1);
		// seems like this is fine so far.
		//qemu_log("previously setup afl, returning\n");
		return;
	}

	has_ran_before = 1;

	if(getenv("AFL_QEMU_INSTRUMENT_ALL")) afl_instrument_all = 1;

	shm_env = getenv("__AFL_SHM_ID");
	if(! shm_env) {
		qemu_log("getenv(__AFL_SHM_ID) is NULL\n");
		return;
	}
	
	shm_id = atoi(shm_env);
	__afl_area_ptr = shmat(shm_id, NULL, 0);
	if((size_t) __afl_area_ptr == -1) {
		qemu_log("afl_setup() - __afl_area_ptr = NULL, bailing\n");
		exit(1);
	}

}

/*
 * forkserver implementation - this is called when the entrypoint (_start)
 * of the binary is reached.
 */

static void afl_forkserver(CPUArchState *env)
{
	char tmp[4];

	memset(tmp, 0, sizeof(tmp));

	if(write(AFL_FORKSERVER_FD + 1, tmp, sizeof(tmp)) != sizeof(tmp)) {
		qemu_log("afl_forkserver() - failed indicating we exist!\n");
		return;
	}

	afl_forkserver_running = 1;

	while(1) {
		pid_t pid;
		u_int32_t wp;
		int status;
		int ff_fd[2];

		if(read(AFL_FORKSERVER_FD, tmp, 4) != 4) {
			qemu_log("afl_forkserver() - failed to read from afl, bailing\n");
			exit(1);
		}

		/*
		 * To implement the fork faster, we need an IPC mechanism between the parent and child.
		 * Surprisingly, using pipes/read/write is pretty good, and probably doesn't need shared
		 * memory and locking etc.
		 */

		if(pipe(ff_fd) == -1) {
			qemu_log("pipe() failed\n");
			exit(1);
		}

		// XXX, we'll assume AFL_FORKFASTER_FD isn't open already.

		if(dup2(ff_fd[0], AFL_FORKFASTER_FD) == -1 || dup2(ff_fd[1], AFL_FORKFASTER_FD+1) == -1) {
			qemu_log("dup2() failed\n");
			exit(1);
		}

		close(ff_fd[0]);
		close(ff_fd[1]);

		pid = fork();

		if(pid == -1) {
			qemu_log("afl_forkserver() - failed to fork(), bailing\n");
			exit(1);
		}

		if(pid == 0) { // child
			close(AFL_FORKSERVER_FD);
			close(AFL_FORKSERVER_FD + 1);

			close(AFL_FORKFASTER_FD);

			return;
		}

		// parent
		close(AFL_FORKFASTER_FD + 1);

		wp = pid;

		if(write(AFL_FORKSERVER_FD + 1, &wp, sizeof(u_int32_t)) != sizeof(u_int32_t)) {
			qemu_log("afl_forkserver() - failed to write pid to parent, bailing\n");
			exit(1);
		}

		afl_fork_faster_reader(env);

		if(waitpid(pid, &status, WUNTRACED) == -1) {
			qemu_log("afl_forkserver() - failed to waitpid(), bailing\n");
			exit(1);
		}

		wp = status;

		if(write(AFL_FORKSERVER_FD + 1, &wp, sizeof(u_int32_t)) != sizeof(u_int32_t)) {
			qemu_log("afl_forkserver() - failed to write() to afl, bailing\n");
			exit(1);
		}

	}

}


inline void afl_maybe_log(abi_ulong cur_loc)
{
	static abi_ulong prev_loc;
	int loc;

	if(__afl_area_ptr == NULL) return;

	if(! afl_instrument_all) {
		if(cur_loc < afl_start_code || cur_loc > afl_end_code) {
			qemu_log("afl_maybe_log(): not instrumenting " TARGET_ABI_FMT_lx "\n", cur_loc);
			return;
		}
	}
	qemu_log("afl_maybe_log(): instrumenting " TARGET_ABI_FMT_lx "\n", cur_loc);

	// TL;DR: the instrumentation does shm_trace_map[cur_loc ^ prev_loc]++

	// We have an abi_ulong.. we need to mash those X bits together to make a suitable 
	// value for AFL instrumentation. Currently, we look at bottom 32 bits, and ignore
	// 64 bit architectures. Some arch instructions may be X bit aligned, so we take 
	// them into account as well.

	cur_loc = ((cur_loc >> 16) ^ (cur_loc >> 2)) ^ (cur_loc & 0xffff);
	cur_loc %= MAP_SIZE;

	loc = cur_loc ^ prev_loc;

	__afl_area_ptr[loc]++;
	prev_loc = cur_loc >> 1;

}

typedef struct afl_ff_struct
{
	target_ulong pc;
	target_ulong cs_base;
	uint64_t flags;
} afl_ff_t;

/*
 * Write the required information to the parent so they can translate the buffer
 * as well, so on next fork() the child doesn't need to redo the translation
 */

void afl_fork_faster_writer(target_ulong pc, target_ulong cs_base, uint64_t flags)
{
	afl_ff_t ff;

	if(! afl_forkserver_running) return;

	ff.pc = pc;
	ff.cs_base = cs_base;
	ff.flags = flags;

	if(write(AFL_FORKFASTER_FD+1, &ff, sizeof(afl_ff_t)) != sizeof(afl_ff_t)) {
		// parent seems to have died?
		qemu_log("parent dead?\n");
		return;
	}
		
}

/*
 * as the child hits untranslated code, it writes the required information to the parent
 * so that the parent can make the next child process run faster
 */

void afl_fork_faster_reader(CPUArchState *env)
{
	afl_ff_t ff;
	// TranslationBlock *tb;

	while(1) {
		if(read(AFL_FORKFASTER_FD, &ff, sizeof(afl_ff_t)) != sizeof(afl_ff_t)) return;

		tb_find_slow(env, ff.pc, ff.cs_base, ff.flags);
		// XXX, keep statistics of this?
	}

	close(AFL_FORKFASTER_FD);
}



int cpu_exec(CPUArchState *env)
{
    CPUState *cpu = ENV_GET_CPU(env);
    CPUClass *cc = CPU_GET_CLASS(cpu);
#ifdef TARGET_I386
    X86CPU *x86_cpu = X86_CPU(cpu);
#endif
    int ret, interrupt_request;
    TranslationBlock *tb;
    uint8_t *tc_ptr;
    uintptr_t next_tb;
    SyncClocks sc;

    /* This must be volatile so it is not trashed by longjmp() */
    volatile bool have_tb_lock = false;

    if(! afl_entry_point) {
        qemu_log("afl_entry_point not set - not expected use case, sorry!\n");
        exit(1);
    }

    if (cpu->halted) {
        if (!cpu_has_work(cpu)) {
            return EXCP_HALTED;
        }

        cpu->halted = 0;
    }

    current_cpu = cpu;

    /* As long as current_cpu is null, up to the assignment just above,
     * requests by other threads to exit the execution loop are expected to
     * be issued using the exit_request global. We must make sure that our
     * evaluation of the global value is performed past the current_cpu
     * value transition point, which requires a memory barrier as well as
     * an instruction scheduling constraint on modern architectures.  */
    smp_mb();

    if (unlikely(exit_request)) {
        cpu->exit_request = 1;
    }

    cc->cpu_exec_enter(cpu);

    /* Calculate difference between guest clock and host clock.
     * This delay includes the delay of the last cycle, so
     * what we have to do is sleep until it is 0. As for the
     * advance/delay we gain here, we try to fix it next time.
     */
    init_delay_params(&sc, cpu);

    /* prepare setjmp context for exception handling */
    for(;;) {
        if (sigsetjmp(cpu->jmp_env, 0) == 0) {
            /* if an exception is pending, we execute it here */
            if (cpu->exception_index >= 0) {
                if (cpu->exception_index >= EXCP_INTERRUPT) {
                    /* exit request from the cpu execution loop */
                    ret = cpu->exception_index;
                    if (ret == EXCP_DEBUG) {
                        cpu_handle_debug_exception(env);
                    }
                    cpu->exception_index = -1;
                    break;
                } else {
#if defined(CONFIG_USER_ONLY)
                    /* if user mode only, we simulate a fake exception
                       which will be handled outside the cpu execution
                       loop */
#if defined(TARGET_I386)
                    cc->do_interrupt(cpu);
#endif
                    ret = cpu->exception_index;
                    cpu->exception_index = -1;
                    break;
#else
                    cc->do_interrupt(cpu);
                    cpu->exception_index = -1;
#endif
                }
            }

            next_tb = 0; /* force lookup of first TB */
            for(;;) {
                interrupt_request = cpu->interrupt_request;
                if (unlikely(interrupt_request)) {
                    if (unlikely(cpu->singlestep_enabled & SSTEP_NOIRQ)) {
                        /* Mask out external interrupts for this step. */
                        interrupt_request &= ~CPU_INTERRUPT_SSTEP_MASK;
                    }
                    if (interrupt_request & CPU_INTERRUPT_DEBUG) {
                        cpu->interrupt_request &= ~CPU_INTERRUPT_DEBUG;
                        cpu->exception_index = EXCP_DEBUG;
                        cpu_loop_exit(cpu);
                    }
                    if (interrupt_request & CPU_INTERRUPT_HALT) {
                        cpu->interrupt_request &= ~CPU_INTERRUPT_HALT;
                        cpu->halted = 1;
                        cpu->exception_index = EXCP_HLT;
                        cpu_loop_exit(cpu);
                    }
#if defined(TARGET_I386)
                    if (interrupt_request & CPU_INTERRUPT_INIT) {
                        cpu_svm_check_intercept_param(env, SVM_EXIT_INIT, 0);
                        do_cpu_init(x86_cpu);
                        cpu->exception_index = EXCP_HALTED;
                        cpu_loop_exit(cpu);
                    }
#else
                    if (interrupt_request & CPU_INTERRUPT_RESET) {
                        cpu_reset(cpu);
                    }
#endif
                    /* The target hook has 3 exit conditions:
                       False when the interrupt isn't processed,
                       True when it is, and we should restart on a new TB,
                       and via longjmp via cpu_loop_exit.  */
                    if (cc->cpu_exec_interrupt(cpu, interrupt_request)) {
                        next_tb = 0;
                    }
                    /* Don't use the cached interrupt_request value,
                       do_interrupt may have updated the EXITTB flag. */
                    if (cpu->interrupt_request & CPU_INTERRUPT_EXITTB) {
                        cpu->interrupt_request &= ~CPU_INTERRUPT_EXITTB;
                        /* ensure that no TB jump will be modified as
                           the program flow was changed */
                        next_tb = 0;
                    }
                }
                if (unlikely(cpu->exit_request)) {
                    cpu->exit_request = 0;
                    cpu->exception_index = EXCP_INTERRUPT;
                    cpu_loop_exit(cpu);
                }
                spin_lock(&tcg_ctx.tb_ctx.tb_lock);
                have_tb_lock = true;
                tb = tb_find_fast(env);
                /* Note: we do it here to avoid a gcc bug on Mac OS X when
                   doing it in tb_find_slow */
                if (tcg_ctx.tb_ctx.tb_invalidated_flag) {
                    /* as some TB could have been invalidated because
                       of memory exceptions while generating the code, we
                       must recompute the hash index here */
                    next_tb = 0;
                    tcg_ctx.tb_ctx.tb_invalidated_flag = 0;
                }

		if(tb->pc == afl_entry_point) {
			afl_setup();
			afl_forkserver(env);
		}

		afl_maybe_log(tb->pc);

                if (qemu_loglevel_mask(CPU_LOG_EXEC)) {
                    qemu_log("Trace %p [" TARGET_FMT_lx "] %s\n",
                             tb->tc_ptr, tb->pc, lookup_symbol(tb->pc));
                }
                /* see if we can patch the calling TB. When the TB
                   spans two pages, we cannot safely do a direct
                   jump. */
                if (next_tb != 0 && tb->page_addr[1] == -1) {
                    tb_add_jump((TranslationBlock *)(next_tb & ~TB_EXIT_MASK),
                                next_tb & TB_EXIT_MASK, tb);
                }
                have_tb_lock = false;
                spin_unlock(&tcg_ctx.tb_ctx.tb_lock);

                /* cpu_interrupt might be called while translating the
                   TB, but before it is linked into a potentially
                   infinite loop and becomes env->current_tb. Avoid
                   starting execution if there is a pending interrupt. */
                cpu->current_tb = tb;
                barrier();
                if (likely(!cpu->exit_request)) {
                    trace_exec_tb(tb, tb->pc);
                    tc_ptr = tb->tc_ptr;
                    /* execute the generated code */
                    next_tb = cpu_tb_exec(cpu, tc_ptr);
                    switch (next_tb & TB_EXIT_MASK) {
                    case TB_EXIT_REQUESTED:
                        /* Something asked us to stop executing
                         * chained TBs; just continue round the main
                         * loop. Whatever requested the exit will also
                         * have set something else (eg exit_request or
                         * interrupt_request) which we will handle
                         * next time around the loop.
                         */
                        tb = (TranslationBlock *)(next_tb & ~TB_EXIT_MASK);
                        next_tb = 0;
                        break;
                    case TB_EXIT_ICOUNT_EXPIRED:
                    {
                        /* Instruction counter expired.  */
                        int insns_left;
                        tb = (TranslationBlock *)(next_tb & ~TB_EXIT_MASK);
                        insns_left = cpu->icount_decr.u32;
                        if (cpu->icount_extra && insns_left >= 0) {
                            /* Refill decrementer and continue execution.  */
                            cpu->icount_extra += insns_left;
                            if (cpu->icount_extra > 0xffff) {
                                insns_left = 0xffff;
                            } else {
                                insns_left = cpu->icount_extra;
                            }
                            cpu->icount_extra -= insns_left;
                            cpu->icount_decr.u16.low = insns_left;
                        } else {
                            if (insns_left > 0) {
                                /* Execute remaining instructions.  */
                                cpu_exec_nocache(env, insns_left, tb);
                                align_clocks(&sc, cpu);
                            }
                            cpu->exception_index = EXCP_INTERRUPT;
                            next_tb = 0;
                            cpu_loop_exit(cpu);
                        }
                        break;
                    }
                    default:
                        break;
                    }
                }
                cpu->current_tb = NULL;
                /* Try to align the host and virtual clocks
                   if the guest is in advance */
                align_clocks(&sc, cpu);
                /* reset soft MMU for next block (it can currently
                   only be set by a memory fault) */
            } /* for(;;) */
        } else {
            /* Reload env after longjmp - the compiler may have smashed all
             * local variables as longjmp is marked 'noreturn'. */
            cpu = current_cpu;
            env = cpu->env_ptr;
            cc = CPU_GET_CLASS(cpu);
            cpu->can_do_io = 1;
#ifdef TARGET_I386
            x86_cpu = X86_CPU(cpu);
#endif
            if (have_tb_lock) {
                spin_unlock(&tcg_ctx.tb_ctx.tb_lock);
                have_tb_lock = false;
            }
        }
    } /* for(;;) */

    cc->cpu_exec_exit(cpu);

    /* fail safe : never use current_cpu outside cpu_exec() */
    current_cpu = NULL;
    return ret;
}
