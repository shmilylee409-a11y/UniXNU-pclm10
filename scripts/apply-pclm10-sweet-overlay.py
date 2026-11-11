#!/usr/bin/env python3
"""Apply OPPO Reno Ace (PCLM10 / SM8150) hardware adaptations onto an ios-sweet patched xnu-8796 tree."""
from __future__ import annotations

import argparse
from pathlib import Path

PCLM10_RESET_C = r'''/*
 * Unattended fastboot reset & UDP control watchdog for OPPO Reno Ace (PCLM10 / SM8150).
 *
 * Pre-maps IMEM restart_reason (0x146bf65c), PM8150 SPMI PON spare, PS_HOLD, and APSS WDT
 * early in arm_init so both normal resets and interrupt-masked panic resets work cleanly
 * without allocations or locks.
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/uio.h>
#include <sys/kpi_socket.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/sched_prim.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t kernel_thread_start(thread_continue_t, void *, thread_t *);
extern void thread_deallocate(thread_t);
extern int hz;
extern uint64_t sweet_usb_rx_count(void);

void pclm10_reset_premap(void);
int pclm10_boot_reset_bootloader(void);
void pclm10_panic_autoreset(void);
void pclm10_watchdog_start(void);
void pclm10_smmu_bypass_stream(uint32_t stream);

enum {
	BR_IMEM_BASE          = 0x146bf000u,
	BR_IMEM_SIZE          = 0x1000u,
	BR_IMEM_REASON_OFF    = 0x65cu,
	BR_REASON_BOOTLOADER  = 0x77665500u,

	BR_PSHOLD_BASE        = 0x0c264000u,
	BR_PSHOLD_SIZE        = 0x1000u,

	BR_WDT_BASE           = 0x17c10000u,
	BR_WDT_SIZE           = 0x1000u,
	BR_WDT_RST            = 0x04u,
	BR_WDT_EN             = 0x08u,
	BR_WDT_BARK           = 0x10u,
	BR_WDT_BITE           = 0x14u,

	SPMI_CORE_BASE        = 0x0c440000u,
	SPMI_CORE_SIZE        = 0x2000u,
	SPMI_CHNLS_BASE       = 0x0c600000u,
	SPMI_CHNL_STRIDE      = 0x10000u,
	SPMI_OBS_BASE         = 0x0e600000u,
	SPMI_OBS_SIZE         = 0x10000u,
	SPMI_OBS_STRIDE       = 0x80u,
	SPMI_CNFG_BASE        = 0x0c40a000u,
	SPMI_CNFG_SIZE        = 0x1000u,

	SPMI_APID_MAP         = 0x900u,
	SPMI_OWNER            = 0x700u,
	SPMI_MAX_APIDS        = 512u,
	SPMI_CMD              = 0x00u,
	SPMI_STATUS           = 0x08u,
	SPMI_WDATA0           = 0x10u,
	SPMI_RDATA0           = 0x18u,
	SPMI_STATUS_DONE      = 0x01u,
	SPMI_STATUS_ERR       = 0x0eu,

	PON_SUBTYPE_ADDR      = 0x805u,
	PON_SPARE_ADDR        = 0x88fu,
	PON_GEN2_SUBTYPE      = 0x04u,
	PON_REASON_BOOTLOADER = 0x02u,

	PSCI_SYSTEM_RESET     = 0x84000009u,
};

static vm_offset_t br_imem, br_pshold, br_wdt;
static vm_offset_t br_spmi_core, br_spmi_cnfg, br_spmi_obs, br_pon_lane;
static unsigned br_spmi_count;
static uint8_t br_pon_subtype;
static bool br_premapped;
static bool br_watchdog_started;
static int br_sleep_chan;

static void
br_udelay(uint32_t usec)
{
	uint64_t freq = 0, start = 0, ticks;
	__asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
	if (!freq || freq > 1000000000ULL) {
		freq = 19200000ULL;
	}
	__asm__ volatile("isb\n\tmrs %0, cntvct_el0" : "=r"(start));
	ticks = (freq * (uint64_t)usec) / 1000000ULL;
	if (!ticks) {
		ticks = 1;
	}
	for (;;) {
		uint64_t now = 0;
		__asm__ volatile("mrs %0, cntvct_el0" : "=r"(now));
		if ((now - start) >= ticks) {
			break;
		}
		__asm__ volatile("yield" ::: "memory");
	}
}

static uint64_t
br_smc_psci(uint64_t fid, uint64_t a1)
{
	register uint64_t x0 __asm__("x0") = fid;
	register uint64_t x1 __asm__("x1") = a1;
	register uint64_t x2 __asm__("x2") = 0;
	register uint64_t x3 __asm__("x3") = 0;
	__asm__ volatile("smc #0"
	    : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
	    :: "memory", "x4", "x5", "x6", "x7", "x8", "x9", "x10",
	       "x11", "x12", "x13", "x14", "x15", "x16", "x17");
	return x0;
}

static uint64_t
br_scm_pwr(uint64_t cmd, uint64_t arg)
{
	const uint64_t fnid = 0x02000000ULL | ((0x9ULL & 0xffULL) << 8) | (cmd & 0xffULL);
	register uint64_t x0 __asm__("x0") = fnid | 0x80000000ULL | 0x40000000ULL;
	register uint64_t x1 __asm__("x1") = 1;
	register uint64_t x2 __asm__("x2") = arg;
	register uint64_t x3 __asm__("x3") = 0;
	__asm__ volatile("smc #0"
	    : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
	    :: "memory", "x4", "x5", "x6", "x7", "x8", "x9", "x10",
	       "x11", "x12", "x13", "x14", "x15", "x16", "x17");
	return x0;
}

static inline uint32_t
br_spmi_rd(vm_offset_t base, unsigned off)
{
	uint32_t v = *(volatile uint32_t *)(base + off);
	__asm__ volatile("dmb oshld" ::: "memory");
	return v;
}

static int
br_spmi_find_apid(unsigned ppid)
{
	if (!br_spmi_core || !br_spmi_cnfg) {
		return -1;
	}
	for (unsigned i = 0; i < br_spmi_count; ++i) {
		uint32_t map = br_spmi_rd(br_spmi_core, SPMI_APID_MAP + 4u * i);
		if (!map || ((map >> 8) & 0xfffu) != ppid) {
			continue;
		}
		if ((br_spmi_rd(br_spmi_cnfg, SPMI_OWNER + 4u * i) & 7u) == 0u) {
			return (int)i;
		}
	}
	return -1;
}

static int
br_spmi_obs_read8(unsigned addr, uint8_t *out)
{
	int apid = br_spmi_find_apid((addr >> 8) & 0xfffu);
	if (apid < 0 || !br_spmi_obs) {
		return 1;
	}
	unsigned off = SPMI_OBS_STRIDE * (unsigned)apid;
	*(volatile uint32_t *)(br_spmi_obs + off + SPMI_CMD) = (1u << 27) | ((addr & 0xffu) << 4);
	__asm__ volatile("dsb osh" ::: "memory");
	for (unsigned i = 0; i < 1000u; ++i) {
		uint32_t st = br_spmi_rd(br_spmi_obs, off + SPMI_STATUS);
		if (st & SPMI_STATUS_DONE) {
			if (st & SPMI_STATUS_ERR) {
				return 2;
			}
			*out = (uint8_t)br_spmi_rd(br_spmi_obs, off + SPMI_RDATA0);
			return 0;
		}
		br_udelay(50);
	}
	return 3;
}

void
pclm10_reset_premap(void)
{
	if (br_premapped) {
		return;
	}
	br_premapped = true;
	br_imem = ml_io_map(BR_IMEM_BASE, BR_IMEM_SIZE);
	br_pshold = ml_io_map(BR_PSHOLD_BASE, BR_PSHOLD_SIZE);
	br_wdt = ml_io_map(BR_WDT_BASE, BR_WDT_SIZE);
	br_spmi_core = ml_io_map(SPMI_CORE_BASE, SPMI_CORE_SIZE);
	br_spmi_cnfg = ml_io_map(SPMI_CNFG_BASE, SPMI_CNFG_SIZE);
	br_spmi_obs = ml_io_map(SPMI_OBS_BASE, SPMI_OBS_SIZE);
	if (br_spmi_core && br_spmi_cnfg && br_spmi_obs) {
		uint32_t ver = br_spmi_rd(br_spmi_core, 0u);
		uint32_t cnt = br_spmi_rd(br_spmi_core, 4u) & 0x7ffu;
		if (ver >= 0x50000000u && ver < 0x70000000u && cnt > 0u && cnt <= SPMI_MAX_APIDS) {
			br_spmi_count = cnt;
			int apid = br_spmi_find_apid((PON_SPARE_ADDR >> 8) & 0xfffu);
			if (apid >= 0) {
				br_pon_lane = ml_io_map(
				    (vm_offset_t)(SPMI_CHNLS_BASE + SPMI_CHNL_STRIDE * (unsigned)apid),
				    SPMI_CHNL_STRIDE);
			}
			(void)br_spmi_obs_read8(PON_SUBTYPE_ADDR, &br_pon_subtype);
		}
	}
}

static void
br_pon_mark_bootloader(void)
{
	if (!br_pon_lane) {
		return;
	}
	unsigned shift = (br_pon_subtype >= PON_GEN2_SUBTYPE) ? 1u : 2u;
	uint8_t mask = (uint8_t)(0xffu << shift);
	uint8_t want = (uint8_t)(PON_REASON_BOOTLOADER << shift);
	uint8_t before = 0;
	(void)br_spmi_obs_read8(PON_SPARE_ADDR, &before);
	uint8_t val = (uint8_t)((before & (uint8_t)~mask) | want);
	*(volatile uint32_t *)(br_pon_lane + SPMI_WDATA0) = val;
	__asm__ volatile("dsb osh" ::: "memory");
	*(volatile uint32_t *)(br_pon_lane + SPMI_CMD) = ((PON_SPARE_ADDR & 0xffu) << 4);
	__asm__ volatile("dsb osh" ::: "memory");
	for (unsigned i = 0; i < 1000u; ++i) {
		uint32_t st = br_spmi_rd(br_pon_lane, SPMI_STATUS);
		if (st & SPMI_STATUS_DONE) {
			break;
		}
		br_udelay(50);
	}
}

int
pclm10_boot_reset_bootloader(void)
{
	if (!br_imem) {
		return 1;
	}
	volatile uint32_t *cell = (volatile uint32_t *)(br_imem + BR_IMEM_REASON_OFF);
	*cell = BR_REASON_BOOTLOADER;
	__asm__ volatile("dsb sy" ::: "memory");
	if (*cell != BR_REASON_BOOTLOADER) {
		return 11;
	}
	br_pon_mark_bootloader();
	(void)br_scm_pwr(0x1ULL, 0ULL);
	(void)br_scm_pwr(0x2ULL, 0ULL);
	(void)br_smc_psci(PSCI_SYSTEM_RESET, 0ULL);
	if (br_pshold) {
		*(volatile uint32_t *)br_pshold = 0u;
		__asm__ volatile("dsb sy" ::: "memory");
	}
	br_udelay(150000u);
	if (br_wdt) {
		*(volatile uint32_t *)(br_wdt + BR_WDT_EN)   = 0u;
		*(volatile uint32_t *)(br_wdt + BR_WDT_RST)  = 1u;
		*(volatile uint32_t *)(br_wdt + BR_WDT_BARK) = 0x400u;
		*(volatile uint32_t *)(br_wdt + BR_WDT_BITE) = 0x800u;
		*(volatile uint32_t *)(br_wdt + BR_WDT_EN)   = 1u;
		*(volatile uint32_t *)(br_wdt + BR_WDT_RST)  = 1u;
		__asm__ volatile("dsb sy" ::: "memory");
		br_udelay(500000u);
	}
	return 99;
}

void
pclm10_panic_autoreset(void)
{
	static bool running;
	unsigned int sec = 0;
	if (!PE_parse_boot_argn("pclm10-panic-reset", &sec, sizeof(sec)) || sec == 0) {
		return;
	}
	if (running) {
		return;
	}
	running = true;
	if (sec > 30u) {
		sec = 30u;
	}
	if (!br_imem) {
		return;
	}
	for (unsigned int i = 0; i < sec * 10u; ++i) {
		br_udelay(100000u);
	}
	(void)pclm10_boot_reset_bootloader();
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
static void
pclm10_watchdog_thread(void *arg __unused, wait_result_t wr __unused)
{
	unsigned int autoreset_sec = 0;
	(void)PE_parse_boot_argn("pclm10-autoreset", &autoreset_sec, sizeof(autoreset_sec));

	socket_t so = NULL;
	bool bound = false;
	uint32_t idle_ticks = 0;
	bool host_held = false;
	char buf[128];

	printf("PCLM10 WATCHDOG: active autoreset=%us udp=5141\n", autoreset_sec);
	for (;;) {
		(void)tsleep(&br_sleep_chan, PZERO, "pclm10wd", hz / 5);

		if (so == NULL && sock_socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP, NULL, NULL, &so) != 0) {
			so = NULL;
		}
		if (so != NULL && !bound) {
			struct sockaddr_in sin;
			bzero(&sin, sizeof(sin));
			sin.sin_len = sizeof(sin);
			sin.sin_family = AF_INET;
			sin.sin_port = htons(5141);
			sin.sin_addr.s_addr = htonl(INADDR_ANY);
			if (sock_bind(so, (const struct sockaddr *)&sin) == 0) {
				bound = true;
			}
		}
		if (so != NULL && bound) {
			for (;;) {
				struct sockaddr_in from;
				bzero(&from, sizeof(from));
				struct iovec iov = { .iov_base = buf, .iov_len = sizeof(buf) - 1 };
				struct msghdr msg = {
					.msg_name = &from,
					.msg_namelen = sizeof(from),
					.msg_iov = &iov,
					.msg_iovlen = 1,
				};
				size_t recvd = 0;
				errno_t err = sock_receive(so, &msg, MSG_DONTWAIT, &recvd);
				if (err != 0 || recvd == 0) {
					break;
				}
				buf[recvd] = '\0';
				host_held = true;
				idle_ticks = 0;
				if (strncmp(buf, "REBOOT", 6) == 0) {
					const char ack[] = "OK REBOOTING\n";
					struct iovec riov = { .iov_base = (void *)(uintptr_t)ack, .iov_len = sizeof(ack) - 1 };
					struct msghdr rmsg = {
						.msg_name = &from,
						.msg_namelen = msg.msg_namelen,
						.msg_iov = &riov,
						.msg_iovlen = 1,
					};
					size_t sent = 0;
					(void)sock_send(so, &rmsg, 0, &sent);
					printf("PCLM10 WATCHDOG: UDP REBOOT requested; resetting to fastboot\n");
					br_udelay(50000u);
					(void)pclm10_boot_reset_bootloader();
				} else if (strncmp(buf, "PING", 4) == 0 || strncmp(buf, "HOLD", 4) == 0) {
					const char ack[] = "OK\n";
					struct iovec riov = { .iov_base = (void *)(uintptr_t)ack, .iov_len = sizeof(ack) - 1 };
					struct msghdr rmsg = {
						.msg_name = &from,
						.msg_namelen = msg.msg_namelen,
						.msg_iov = &riov,
						.msg_iovlen = 1,
					};
					size_t sent = 0;
					(void)sock_send(so, &rmsg, 0, &sent);
				}
			}
		}

		if (sweet_usb_rx_count() > 0) {
			host_held = true;
		}
		if (autoreset_sec > 0 && !host_held && ++idle_ticks >= autoreset_sec * 5u) {
			printf("PCLM10 AUTORESET: no USB host after %us; rebooting to fastboot\n", autoreset_sec);
			br_udelay(50000u);
			(void)pclm10_boot_reset_bootloader();
			break;
		}
	}
}
#pragma clang diagnostic pop

void
pclm10_watchdog_start(void)
{
	if (br_watchdog_started) {
		return;
	}
	br_watchdog_started = true;
	pclm10_reset_premap();
	thread_t thread = THREAD_NULL;
	if (kernel_thread_start(pclm10_watchdog_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

void
pclm10_smmu_bypass_stream(uint32_t stream)
{
	static vm_offset_t smmu;
	stream &= 0x7fffu;
	if (smmu == 0) {
		smmu = ml_io_map(0x15000000ULL, 0xd0000u);
	}
	if (smmu == 0) {
		printf("PCLM10 SMMU: map failed for stream=0x%x\n", stream);
		return;
	}
	uint32_t idr0 = *(volatile uint32_t *)(smmu + 0x20);
	uint32_t idr1 = *(volatile uint32_t *)(smmu + 0x24);
	unsigned nsmr = idr0 & 0xffu;
	unsigned ncb = idr1 & 0xffu;
	unsigned pages = 1u << (((idr1 >> 28) & 7u) + 1u);
	unsigned s2 = (idr1 >> 16) & 0xffu;
	unsigned slot = 0xffffu;
	bool used_bank[256];
	for (unsigned i = 0; i < 256u; ++i) {
		used_bank[i] = false;
	}
	for (unsigned i = 0; i < nsmr && i < 256u; ++i) {
		uint32_t smr = *(volatile uint32_t *)(smmu + 0x800u + 4u * i);
		uint32_t s2cr = *(volatile uint32_t *)(smmu + 0xc00u + 4u * i);
		if (smr & 0x80000000u) {
			unsigned mask = (smr >> 16) & 0x7fffu;
			if (((((smr ^ stream) & 0x7fffu) & ~mask) == 0u) && slot == 0xffffu) {
				slot = i;
			}
			if (((s2cr >> 16) & 3u) == 0u) {
				unsigned b = s2cr & 0xffu;
				if (b < 256u) {
					used_bank[b] = true;
				}
			}
		} else if (slot == 0xffffu) {
			slot = i;
		}
	}
	unsigned bank = 0xffffu;
	if (slot != 0xffffu) {
		uint32_t cur_smr = *(volatile uint32_t *)(smmu + 0x800u + 4u * slot);
		uint32_t cur_s2cr = *(volatile uint32_t *)(smmu + 0xc00u + 4u * slot);
		if ((cur_smr & 0x80000000u) && (((cur_s2cr >> 16) & 3u) == 0u)) {
			bank = cur_s2cr & 0xffu;
		} else {
			for (unsigned b = s2; b < ncb && b < 256u; ++b) {
				if (!used_bank[b]) {
					bank = b;
					break;
				}
			}
		}
	}
	if (slot == 0xffffu || bank == 0xffffu) {
		printf("PCLM10 SMMU: no free slot/bank stream=0x%x\n", stream);
		return;
	}
	vm_offset_t cb = smmu + (vm_offset_t)(pages + bank) * 4096u;
	*(volatile uint32_t *)(cb + 0x0) = 0u;
	__asm__ volatile("dsb sy" ::: "memory");
	*(volatile uint32_t *)(smmu + 0x1000u + 4u * bank) = 0x00010000u; /* CBAR type=1 bypass */
	*(volatile uint32_t *)(smmu + 0x1800u + 4u * bank) |= 1u;         /* CBA2R VA64 */
	*(volatile uint32_t *)(smmu + 0xc00u + 4u * slot) = bank;         /* S2CR type=0 CB */
	__asm__ volatile("dsb sy" ::: "memory");
	*(volatile uint32_t *)(smmu + 0x800u + 4u * slot) = 0x80000000u | stream;
	__asm__ volatile("dsb sy; isb" ::: "memory");
	printf("PCLM10 SMMU: stream 0x%x -> slot=%u bank=%u cbar=0x%08x sctlr=0x%08x\n",
	    stream, slot, bank,
	    *(volatile uint32_t *)(smmu + 0x1000u + 4u * bank),
	    *(volatile uint32_t *)(cb + 0x0));
}
#endif /* ARM64_BOARD_CONFIG_SWEET */
'''


def replace_exact(path: Path, old: str, new: str, label: str) -> None:
    text = path.read_text(encoding='utf-8')
    cnt = text.count(old)
    if cnt != 1:
        raise SystemExit(f"{path}: expected 1 match for {label}, found {cnt}")
    path.write_text(text.replace(old, new, 1), encoding='utf-8')


def apply_overlay(xnu: Path) -> None:
    # 1. Early framebuffer physical address: 0x9c000000 -> 0x9d000000 (within same 32MB L2 block)
    replace_exact(
        xnu / 'osfmk/arm64/start.s',
        'MOV64\tx0, (0x9c000000 + 2368 * 4320)',
        'MOV64\tx0, (0x9d000000 + 2368 * 4320)',
        'start.s stage0 fb address',
    )
    replace_exact(
        xnu / 'osfmk/arm64/sweet_early.c',
        'static volatile uint32_t *early_fb = (volatile uint32_t *)(uintptr_t)0x9c000000;',
        'static volatile uint32_t *early_fb = (volatile uint32_t *)(uintptr_t)0x9d000000;',
        'sweet_early.c initial fb pointer',
    )
    replace_exact(
        xnu / 'osfmk/arm64/arm_vm_init.c',
        'sweet_early_relocate(SWEET_EARLY_FB_VA);',
        'sweet_early_relocate(SWEET_EARLY_FB_VA + 0x01000000ULL);',
        'arm_vm_init sweet_early_relocate',
    )

    # 2. Pre-map PCLM10 reset registers immediately after arm_vm_init & hook ALL panic stop paths
    replace_exact(
        xnu / 'osfmk/arm/arm_init.c',
        '\tarm_vm_init(xmaxmem, args);',
        '\tarm_vm_init(xmaxmem, args);\n#if defined(SWEET)\n\textern void pclm10_reset_premap(void);\n\tpclm10_reset_premap();\n#endif',
        'arm_init pclm10_reset_premap',
    )
    replace_exact(
        xnu / 'osfmk/kern/debug.c',
        '\t\t/* If panic debugging is configured and we\'re on a dev fused device, spin for astris to connect */\n\t\tpanic_spin_shmcon();',
        '#if defined(SWEET)\n\t\textern void pclm10_panic_autoreset(void);\n\t\tpclm10_panic_autoreset();\n#endif\n\t\t/* If panic debugging is configured and we\'re on a dev fused device, spin for astris to connect */\n\t\tpanic_spin_shmcon();',
        'debug.c panic_spin_shmcon hook',
    )
    replace_exact(
        xnu / 'osfmk/kern/debug.c',
        '\tif (!panicDebugging) {\n\t\tkdp_machine_reboot_type(kPEPanicRestartCPU, debugger_panic_options);\n\t}',
        '\tif (!panicDebugging) {\n#if defined(SWEET)\n\t\textern void pclm10_panic_autoreset(void);\n\t\tpclm10_panic_autoreset();\n#endif\n\t\tkdp_machine_reboot_type(kPEPanicRestartCPU, debugger_panic_options);\n\t}',
        'debug.c kdp_machine_reboot_type hook',
    )
    replace_exact(
        xnu / 'osfmk/kern/debug.c',
        'void\npanic_spin_forever(void)\n{\n\tfor (;;) {',
        'void\npanic_spin_forever(void)\n{\n#if defined(SWEET)\n\textern void pclm10_panic_autoreset(void);\n\tpclm10_panic_autoreset();\n#endif\n\tfor (;;) {',
        'debug.c panic_spin_forever hook',
    )

    # 3. Add bsd/dev/pclm10_reset.c
    (xnu / 'bsd/dev/pclm10_reset.c').write_text(PCLM10_RESET_C, encoding='utf-8')
    replace_exact(
        xnu / 'bsd/conf/files',
        'bsd/dev/sweet_usb.c\t\t\tstandard',
        'bsd/dev/sweet_usb.c\t\t\tstandard\nbsd/dev/pclm10_reset.c\t\t\tstandard',
        'bsd/conf/files pclm10_reset.c',
    )

    # 4. Adapt sweet_usb.c for SM8150 GCC clocks, QSCRATCH UTMI-as-PIPE, and PHY restore across CSFTRST
    sweet_usb = xnu / 'bsd/dev/sweet_usb.c'
    replace_exact(
        sweet_usb,
        '#define GCC_SIZE                0x00090000\n#define GCC_USB30_PRIM_GDSCR    0x0f004         /* bit 31: power on */\n#define CBCR_CLK_OFF            (1u << 31)\n\nstatic const struct {\n\tconst char *name;\n\tuint32_t    off;\n} usb_cbcr[] = {\n\t{ "master", 0x0f010 },\n\t{ "sleep", 0x0f014 },\n\t{ "utmi", 0x0f018 },\n\t{ "cfg-noc-axi", 0x0502c },\n\t{ "aggre-axi", 0x8201c },\n\t{ "ahb2phy", 0x6a004 },\n\t{ "clkref", 0x8c010 },\n};',
        '#define GCC_SIZE                0x001f0000\n#define GCC_USB30_PRIM_GDSCR    0x0f004         /* bit 31: power on */\n#define CBCR_CLK_OFF            (1u << 31)\n\nstatic const struct {\n\tconst char *name;\n\tuint32_t    off;\n} usb_cbcr[] = {\n\t{ "master", 0x0f010 },\n\t{ "sleep", 0x0f014 },\n\t{ "utmi", 0x0f018 },\n\t{ "phy_aux", 0x0f050 },\n\t{ "phy_com_aux", 0x0f054 },\n};',
        'sweet_usb.c SM8150 GCC table',
    )
    replace_exact(
        sweet_usb,
        'void sweet_usb_probe(void);',
        'void sweet_usb_probe(void);\nuint64_t sweet_usb_rx_count(void);\nextern void pclm10_smmu_bypass_stream(uint32_t stream);\nstatic bool pclm10_usb_probed;',
        'sweet_usb.c declarations',
    )
    replace_exact(
        sweet_usb,
        '\tif (enabled >= 2) {\n\t\tsweet_usb_gadget_start(dwc);\n\t}',
        '\tif (enabled >= 2) {\n\t\tuint32_t usb_stream = 0x140u;\n\t\tPE_parse_boot_argn("usbstream", &usb_stream, sizeof(usb_stream));\n\t\tpclm10_smmu_bypass_stream(usb_stream);\n\t\tsweet_usb_gadget_start(dwc);\n\t}',
        'sweet_usb.c SMMU stream 0x140 bypass',
    )
    replace_exact(
        sweet_usb,
        '\tPE_parse_boot_argn("sweetusb", &enabled, sizeof(enabled));\n\tif (!enabled) {\n\t\treturn;\n\t}',
        '\tif (pclm10_usb_probed) {\n\t\treturn;\n\t}\n\tpclm10_usb_probed = true;\n\tPE_parse_boot_argn("sweetusb", &enabled, sizeof(enabled));\n\tif (!enabled) {\n\t\treturn;\n\t}',
        'sweet_usb.c idempotent guard',
    )
    replace_exact(
        sweet_usb,
        '\tclocks_on = (gdscr & (1u << 31)) && !(cbcr[0] & CBCR_CLK_OFF) && !(cbcr[3] & CBCR_CLK_OFF);',
        '\tclocks_on = (gdscr & (1u << 31)) && !(cbcr[0] & CBCR_CLK_OFF);',
        'sweet_usb.c SM8150 clock gate',
    )
    replace_exact(
        sweet_usb,
        '\tif (cbcr[5] & CBCR_CLK_OFF) {\n\t\tprintf("SWEET USB: ahb2phy clock off; QUSB2 PHY not read\\n");\n\t\treturn;\n\t}',
        '\treturn; /* skip SM7150 ahb2phy QUSB2 probe on SM8150 */',
        'sweet_usb.c skip SM7150 QUSB2 cbcr[5]',
    )
    replace_exact(
        sweet_usb,
        '#define QSCRATCH_PA             0x0a6f8800\n#define QSCRATCH_HS_PHY_CTRL    0x10',
        '#define QSCRATCH_PA             0x0a6f8800\n#define QSCRATCH_GENERAL_CFG    0x08\n#define QSCRATCH_HS_PHY_CTRL    0x10',
        'sweet_usb.c QSCRATCH_GENERAL_CFG',
    )
    replace_exact(
        sweet_usb,
        '\t/* Device controller soft reset (PHY and global setup stay as ABL left them). */\n\tdwc_wr(DCTL, DCTL_CSFTRST);\n\tint i;\n\tfor (i = 0; i < 500 && (dwc_rd(DCTL) & DCTL_CSFTRST); i++) {\n\t\tdelay(1000);\n\t}',
        '\t/* PCLM10 SM8150 HS-only: preserve PHY regs and route UTMI as PIPE clock across CSFTRST. */\n\tuint32_t saved_phy2 = dwc_rd(0xc200);\n\tuint32_t saved_phy3 = dwc_rd(0xc2c0);\n\tvm_offset_t qs_pre = ml_io_map(QSCRATCH_PA, 0x100);\n\tif (qs_pre != 0) {\n\t\tuint32_t gcfg = usb_rd(qs_pre, QSCRATCH_GENERAL_CFG);\n\t\t*(volatile uint32_t *)(qs_pre + QSCRATCH_GENERAL_CFG) = gcfg | (1u << 8);\n\t\t__asm__ volatile("dsb sy" ::: "memory");\n\t\tdelay(200);\n\t\tgcfg = (gcfg | (1u << 8) | (1u << 0) | (1u << 3));\n\t\t*(volatile uint32_t *)(qs_pre + QSCRATCH_GENERAL_CFG) = gcfg;\n\t\t__asm__ volatile("dsb sy" ::: "memory");\n\t\tdelay(200);\n\t\t*(volatile uint32_t *)(qs_pre + QSCRATCH_GENERAL_CFG) = gcfg & ~(1u << 8);\n\t\t__asm__ volatile("dsb sy" ::: "memory");\n\t}\n\tdwc_wr(DCTL, DCTL_CSFTRST);\n\tint i;\n\tfor (i = 0; i < 500 && (dwc_rd(DCTL) & DCTL_CSFTRST); i++) {\n\t\tdelay(1000);\n\t}\n\tfor (int k = 0; k < 50; k++) {\n\t\tdelay(1000);\n\t}\n\tdwc_wr(0xc2c0, saved_phy3 & ~((1u << 17) | (1u << 27) | (1u << 31)));\n\tdwc_wr(0xc200, saved_phy2 & ~((1u << 6) | (1u << 8) | (1u << 31)));\n\tdelay(1000);\n\tif (qs_pre != 0) {\n\t\tuint32_t gcfg = usb_rd(qs_pre, QSCRATCH_GENERAL_CFG);\n\t\t*(volatile uint32_t *)(qs_pre + QSCRATCH_GENERAL_CFG) = (gcfg | (1u << 0) | (1u << 3)) & ~(1u << 8);\n\t\t__asm__ volatile("dsb sy" ::: "memory");\n\t}\n\tdwc_wr(GCTL, dwc_rd(GCTL) | 1u); /* DWC3_GCTL_DSBLCLKGTNG */',
        'sweet_usb.c SM8150 CSFTRST + QSCRATCH UTMI-PIPE',
    )
    # Append sweet_usb_rx_count() cleanly
    text = sweet_usb.read_text(encoding='utf-8')
    anchor = '#endif /* ARM64_BOARD_CONFIG_SWEET */'
    if text.count(anchor) != 1:
        raise SystemExit('sweet_usb.c missing end #endif')
    sweet_usb.write_text(
        text.replace(
            anchor,
            'uint64_t\nsweet_usb_rx_count(void)\n{\n\treturn ud.rx_frames;\n}\n' + anchor,
            1,
        ),
        encoding='utf-8',
    )

    # 5. Adapt sweet_ufs.c: SM8150 SMMU stream 0x300 bypass + opt-in SM7150 keys/touch
    sweet_ufs = xnu / 'bsd/dev/sweet_ufs.c'
    replace_exact(
        sweet_ufs,
        'static int\nufs_take_over_host(void)',
        'extern void pclm10_smmu_bypass_stream(uint32_t stream);\n\nstatic int\nufs_take_over_host(void)',
        'sweet_ufs.c SMMU declaration',
    )
    replace_exact(
        sweet_ufs,
        '\tlck_mtx_lock(&sweet_ufs_lock);\n\terror = ufs_take_over_host();',
        '\t{\n\t\tuint32_t ufs_stream = 0x300u;\n\t\tPE_parse_boot_argn("ufsstream", &ufs_stream, sizeof(ufs_stream));\n\t\tpclm10_smmu_bypass_stream(ufs_stream);\n\t}\n\tlck_mtx_lock(&sweet_ufs_lock);\n\terror = ufs_take_over_host();',
        'sweet_ufs.c call SMMU bypass',
    )
    replace_exact(
        sweet_ufs,
        '\t/* Input drivers (HID services in iokit/Kernel/arm/hid/SweetHID.cpp). */\n\tsweet_keys_init();\n\tsweet_touch_init();',
        '\t/* Input drivers: disabled by default on PCLM10 SM8150 until S3706/PM8150 support is wired. */\n\tuint32_t enable_keys = 0, enable_touch = 0;\n\tPE_parse_boot_argn("sweetkeys", &enable_keys, sizeof(enable_keys));\n\tPE_parse_boot_argn("sweettouch", &enable_touch, sizeof(enable_touch));\n\tif (enable_keys) sweet_keys_init();\n\tif (enable_touch) sweet_touch_init();',
        'sweet_ufs.c gate SM7150 keys/touch',
    )

    # 6. Start USB ECM + watchdog early in IOFindBSDRoot (works for both rd=md0 and rd=ufsroot0)
    replace_exact(
        xnu / 'iokit/bsddev/IOKitBSDInit.cpp',
        '\tif (strcmp(rdBootVar, "ufsroot0") == 0) {',
        '\t{\n\t\textern void pclm10_watchdog_start(void);\n\t\textern void sweet_usb_probe(void);\n\t\tpclm10_watchdog_start();\n\t\tsweet_usb_probe();\n\t}\n\tif (strcmp(rdBootVar, "ufsroot0") == 0) {',
        'IOKitBSDInit start USB and watchdog',
    )

    # 7. Match oppo,pclm10 in KernelConfigTables
    replace_exact(
        xnu / 'iokit/KernelConfigTables.cpp',
        "    \"     'IONameMatch'     = 'xiaomi,sweet';\"\n",
        "    \"     'IONameMatch'     = 'xiaomi,sweet';\"\n"
        "    \"     'IOProbeScore'    = 10000:32;\"\n"
        "    \"   },\"\n"
        "    \"   {\"\n"
        "    \"     'IOClass'         = SweetPlatformExpert;\"\n"
        "    \"     'IOProviderClass' = IOPlatformExpertDevice;\"\n"
        "    \"     'IONameMatch'     = 'oppo,pclm10';\"\n",
        'KernelConfigTables match oppo,pclm10',
    )

    # 8. Xcode 16 iig compatibility for xnu-8796 (adds kernelContent to struct IORPC)
    old_iorpc = (
        'struct IORPC {\n'
        '\tIORPCMessageMach * message;\n'
        '\tIORPCMessageMach * reply;\n'
        '\tuint32_t           sendSize;\n'
        '\tuint32_t           replySize;\n'
        '};'
    )
    new_iorpc = (
        'struct IORPC {\n'
        '\tIORPCMessageMach * message;\n'
        '\tIORPCMessageMach * reply;\n'
        '\tuint32_t           sendSize;\n'
        '\tuint32_t           replySize;\n'
        '#ifdef KERNEL\n'
        '\tIORPCMessage     * kernelContent;\n'
        '#endif /* KERNEL */\n'
        '};'
    )
    for rel in ('iokit/DriverKit/IORPC.h', 'iokit/IOKit/IORPC.h'):
        p = xnu / rel
        if 'kernelContent' not in p.read_text(encoding='utf-8'):
            replace_exact(p, old_iorpc, new_iorpc, f'{rel} IORPC.kernelContent')
    iouserserver = xnu / 'iokit/Kernel/IOUserServer.cpp'
    if '.kernelContent = message' not in iouserserver.read_text(encoding='utf-8'):
        replace_exact(
            iouserserver,
            'IORPC rpc = { .message = msgin, .reply = msgout, .sendSize = msgin->msgh.msgh_size, .replySize = replyAlloc };',
            'IORPC rpc = { .message = msgin, .reply = msgout, .sendSize = msgin->msgh.msgh_size, .replySize = replyAlloc, .kernelContent = message };',
            'IOUserServer kernelDispatch kernelContent',
        )
        replace_exact(
            iouserserver,
            'rpc.replySize = ((uint32_t) (sizeof(buffer.buffer) - inSize));    // inSize was checked',
            'rpc.replySize = ((uint32_t) (sizeof(buffer.buffer) - inSize));    // inSize was checked\n\trpc.kernelContent = message;',
            'IOUserServer rpc kernelContent',
        )
    print(f'Applied PCLM10 SM8150 overlay cleanly onto {xnu}')


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--xnu', type=Path, required=True, help='Path to patched ios-sweet xnu tree')
    args = p.parse_args()
    apply_overlay(args.xnu)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
