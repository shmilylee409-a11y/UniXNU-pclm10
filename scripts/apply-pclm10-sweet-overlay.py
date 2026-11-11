#!/usr/bin/env python3
"""Apply OPPO Reno Ace (PCLM10 / SM8150) hardware adaptations onto an ios-sweet patched xnu-8796 tree."""
from __future__ import annotations

import argparse
import shutil
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
extern vm_offset_t ml_io_map_unmappable(vm_offset_t phys_addr, vm_size_t size, uint32_t flags);
extern kern_return_t kernel_thread_start(thread_continue_t, void *, thread_t *);
extern void thread_deallocate(thread_t);
extern int hz;
extern uint64_t sweet_usb_rx_count(void);

void pclm10_reset_premap(void);
int pclm10_boot_reset_bootloader(void);
void pclm10_panic_autoreset(void);
void pclm10_early_panic_reset(volatile uint32_t *imem);
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
	uint64_t max_spins = (uint64_t)usec * 250ULL + 2000ULL;
	for (uint64_t spins = 0; spins < max_spins; ++spins) {
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
	br_imem = ml_io_map_unmappable(BR_IMEM_BASE, BR_IMEM_SIZE, 7u);
	if (!br_imem) {
		br_imem = ml_io_map(BR_IMEM_BASE, BR_IMEM_SIZE);
	}
	br_pshold = ml_io_map_unmappable(BR_PSHOLD_BASE, BR_PSHOLD_SIZE, 7u);
	if (!br_pshold) {
		br_pshold = ml_io_map(BR_PSHOLD_BASE, BR_PSHOLD_SIZE);
	}
	br_wdt = ml_io_map_unmappable(BR_WDT_BASE, BR_WDT_SIZE, 7u);
	if (!br_wdt) {
		br_wdt = ml_io_map(BR_WDT_BASE, BR_WDT_SIZE);
	}
	br_spmi_core = ml_io_map_unmappable(SPMI_CORE_BASE, SPMI_CORE_SIZE, 7u);
	br_spmi_cnfg = ml_io_map_unmappable(SPMI_CNFG_BASE, SPMI_CNFG_SIZE, 7u);
	br_spmi_obs = ml_io_map_unmappable(SPMI_OBS_BASE, SPMI_OBS_SIZE, 7u);
	if (br_spmi_core && br_spmi_cnfg && br_spmi_obs) {
		uint32_t ver = br_spmi_rd(br_spmi_core, 0u);
		uint32_t cnt = br_spmi_rd(br_spmi_core, 4u) & 0x7ffu;
		if (ver >= 0x50000000u && ver < 0x70000000u && cnt > 0u && cnt <= SPMI_MAX_APIDS) {
			br_spmi_count = cnt;
			int apid = br_spmi_find_apid((PON_SPARE_ADDR >> 8) & 0xfffu);
			if (apid >= 0) {
				br_pon_lane = ml_io_map_unmappable(
				    (vm_offset_t)(SPMI_CHNLS_BASE + SPMI_CHNL_STRIDE * (unsigned)apid),
				    SPMI_CHNL_STRIDE, 7u);
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
	if (br_imem) {
		volatile uint32_t *cell = (volatile uint32_t *)(br_imem + BR_IMEM_REASON_OFF);
		*cell = BR_REASON_BOOTLOADER;
		__asm__ volatile("dsb sy" ::: "memory");
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

void
pclm10_early_panic_reset(volatile uint32_t *imem)
{
	static bool early_running;
	if (early_running) {
		return;
	}
	early_running = true;
	unsigned int sec = 8u;
	if (PE_state.initialized) {
		unsigned int parsed = 0;
		if (PE_parse_boot_argn("pclm10-panic-reset", &parsed, sizeof(parsed))) {
			if (parsed == 0) {
				return;
			}
			sec = parsed > 30u ? 30u : parsed;
		}
	}
	for (unsigned int i = 0; i < sec * 10u; ++i) {
		br_udelay(100000u);
	}
	if (imem != NULL) {
		imem[BR_IMEM_REASON_OFF / 4u] = BR_REASON_BOOTLOADER;
		__asm__ volatile("dsb sy" ::: "memory");
	}
	(void)pclm10_boot_reset_bootloader();
	(void)br_scm_pwr(0x1ULL, 0ULL);
	(void)br_scm_pwr(0x2ULL, 0ULL);
	(void)br_smc_psci(PSCI_SYSTEM_RESET, 0ULL);
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
    #    and early IMEM (0x14000000) + LowExceptionVectorBase reset for pre-arm_vm_init faults
    replace_exact(
        xnu / 'osfmk/arm64/start.s',
        'MOV64\tx0, (0x9c000000 + 2368 * 4320)',
        'MOV64\tx0, (0x9d000000 + 2368 * 4320)',
        'start.s stage0 fb address',
    )
    replace_exact(
        xnu / 'osfmk/arm64/start.s',
        '\tstr\t\tx11, [x13, x12, lsl #TTE_SHIFT]\n#endif',
        '\tstr\t\tx11, [x13, x12, lsl #TTE_SHIFT]\n'
        '\t/* PCLM10: map 0x14000000 (IMEM 0x146bf65c) Device-nGnRnE in V=P L2 */\n'
        '\tMOV64\tx10, 0x14000000\n'
        '\tMOV64\tx11, (ARM_TTE_TYPE_BLOCK | ARM_TTE_VALID | ARM_TTE_BLOCK_SH(SH_OUTER_MEMORY) | ARM_TTE_BLOCK_ATTRINDX(CACHE_ATTRINDX_DISABLE) | ARM_TTE_BLOCK_AF | ARM_TTE_BLOCK_PNX | ARM_TTE_BLOCK_NX)\n'
        '\torr\t\tx11, x11, x10\n'
        '\tand\t\tx12, x10, #(ARM_TT_L2_INDEX_MASK)\n'
        '\tlsr\t\tx12, x12, #(ARM_TT_L2_SHIFT)\n'
        '\tstr\t\tx11, [x13, x12, lsl #TTE_SHIFT]\n#endif',
        'start.s early IMEM V=P mapping',
    )
    low_vec_old = (
        'LEXT(LowExceptionVectorBase)\n'
        '\t/* EL1 SP 0 */\n'
        '\tb\t\t.\n'
        '\t.align\t7\n'
        '\tb\t\t.\n'
        '\t.align\t7\n'
        '\tb\t\t.\n'
        '\t.align\t7\n'
        '\tb\t\t.\n'
        '\t/* EL1 SP1 */\n'
        '\t.align\t7\n'
        '\tb\t\t.\n'
        '\t.align\t7\n'
        '\tb\t\t.\n'
        '\t.align\t7\n'
        '\tb\t\t.\n'
        '\t.align\t7\n'
        '\tb\t\t.'
    )
    low_vec_reset = (
        '\tMOV64\tx0, 0x146bf65c\n'
        '\tmovz\tw1, #0x5500\n'
        '\tmovk\tw1, #0x7766, lsl #16\n'
        '\tstr\t\tw1, [x0]\n'
        '\tdsb\t\tsy\n'
        '\tMOV64\tx0, 0x84000009\n'
        '\tmov\t\tx1, #0\n'
        '\tmov\t\tx2, #0\n'
        '\tmov\t\tx3, #0\n'
        '\tsmc\t\t#0\n'
        '\tb\t\t.'
    )
    low_vec_new = (
        'LEXT(LowExceptionVectorBase)\n'
        '\t/* EL1 SP 0 */\n'
        + low_vec_reset + '\n'
        '\t.align\t7\n'
        + low_vec_reset + '\n'
        '\t.align\t7\n'
        + low_vec_reset + '\n'
        '\t.align\t7\n'
        + low_vec_reset + '\n'
        '\t/* EL1 SP1 */\n'
        '\t.align\t7\n'
        + low_vec_reset + '\n'
        '\t.align\t7\n'
        + low_vec_reset + '\n'
        '\t.align\t7\n'
        + low_vec_reset + '\n'
        '\t.align\t7\n'
        + low_vec_reset
    )
    replace_exact(xnu / 'osfmk/arm64/start.s', low_vec_old, low_vec_new, 'start.s LowExceptionVectorBase reset')
    replace_exact(
        xnu / 'osfmk/arm64/sweet_early.c',
        'static volatile uint32_t *early_fb = (volatile uint32_t *)(uintptr_t)0x9c000000;',
        'static volatile uint32_t *early_fb = (volatile uint32_t *)(uintptr_t)0x9d000000;\nstatic volatile uint32_t *early_imem = (volatile uint32_t *)(uintptr_t)0x146bf000;\nextern void pclm10_early_panic_reset(volatile uint32_t *imem);',
        'sweet_early.c initial fb and imem pointers',
    )
    replace_exact(
        xnu / 'osfmk/arm64/sweet_early.c',
        '\tearly_fb = (volatile uint32_t *)(uintptr_t)base;\n\t__asm__ volatile("dsb sy" ::: "memory");',
        '\tearly_fb = (volatile uint32_t *)(uintptr_t)base;\n\tearly_imem = (volatile uint32_t *)(uintptr_t)(base + 0x01000000ULL + 0x006bf000ULL);\n\t__asm__ volatile("dsb sy" ::: "memory");',
        'sweet_early.c relocate imem',
    )
    replace_exact(
        xnu / 'osfmk/arm64/sweet_early.c',
        '\tsweet_early_putc(\'\\n\');\n\t__asm__ volatile("dsb sy" ::: "memory");\n}',
        '\tsweet_early_putc(\'\\n\');\n\t__asm__ volatile("dsb sy" ::: "memory");\n\tpclm10_early_panic_reset(early_imem);\n}',
        'sweet_early.c early panic autoreset',
    )
    replace_exact(
        xnu / 'osfmk/arm64/arm_vm_init.c',
        '#define SWEET_EARLY_FB_VA (VM_MIN_KERNEL_ADDRESS + ARM_TT_L2_SIZE)',
        '#define SWEET_EARLY_FB_VA   (VM_MIN_KERNEL_ADDRESS + ARM_TT_L2_SIZE)\n#define SWEET_EARLY_IMEM_VA (VM_MIN_KERNEL_ADDRESS + 2 * ARM_TT_L2_SIZE)',
        'arm_vm_init SWEET_EARLY_IMEM_VA',
    )
    replace_exact(
        xnu / 'osfmk/arm64/arm_vm_init.c',
        '\t*slot = 0x9c000000ULL | ARM_TTE_VALID | ARM_TTE_TYPE_BLOCK |\n'
        '\t    ARM_TTE_BLOCK_SH(SH_OUTER_MEMORY) |\n'
        '\t    ARM_TTE_BLOCK_ATTRINDX(CACHE_ATTRINDX_WRITECOMB) |\n'
        '\t    ARM_TTE_BLOCK_AF | ARM_TTE_BLOCK_PNX | ARM_TTE_BLOCK_NX;\n'
        '\t__asm__ volatile("dsb ishst" ::: "memory");',
        '\t*slot = 0x9c000000ULL | ARM_TTE_VALID | ARM_TTE_TYPE_BLOCK |\n'
        '\t    ARM_TTE_BLOCK_SH(SH_OUTER_MEMORY) |\n'
        '\t    ARM_TTE_BLOCK_ATTRINDX(CACHE_ATTRINDX_WRITECOMB) |\n'
        '\t    ARM_TTE_BLOCK_AF | ARM_TTE_BLOCK_PNX | ARM_TTE_BLOCK_NX;\n'
        '\tassert(SWEET_EARLY_IMEM_VA + ARM_TT_L2_SIZE <= gVirtBase);\n'
        '\ttt_entry_t *imem_slot = &l2[(SWEET_EARLY_IMEM_VA & ARM_TT_L2_INDEX_MASK) >> ARM_TT_L2_SHIFT];\n'
        '\tassert(*imem_slot == ARM_TTE_EMPTY);\n'
        '\t*imem_slot = 0x14000000ULL | ARM_TTE_VALID | ARM_TTE_TYPE_BLOCK |\n'
        '\t    ARM_TTE_BLOCK_SH(SH_OUTER_MEMORY) |\n'
        '\t    ARM_TTE_BLOCK_ATTRINDX(CACHE_ATTRINDX_DISABLE) |\n'
        '\t    ARM_TTE_BLOCK_AF | ARM_TTE_BLOCK_PNX | ARM_TTE_BLOCK_NX;\n'
        '\t__asm__ volatile("dsb ishst" ::: "memory");',
        'arm_vm_init map SWEET_EARLY_IMEM_VA',
    )
    replace_exact(
        xnu / 'osfmk/arm64/arm_vm_init.c',
        'sweet_early_relocate(SWEET_EARLY_FB_VA);',
        'sweet_early_relocate(SWEET_EARLY_FB_VA + 0x01000000ULL);',
        'arm_vm_init sweet_early_relocate',
    )
    replace_exact(
        xnu / 'osfmk/arm64/arm_vm_init.c',
        '\tva_l1_end = (va_l1_end + 0x00000000007FFFFFULL) & 0xFFFFFFFFFF800000ULL;',
        '\tva_l1_end = (va_l1_end + 0x00000000007FFFFFULL) & 0xFFFFFFFFFF800000ULL;\n'
        '#if defined(SWEET)\n'
        '\tif (va_l1_end < va_l1 + (128ULL << 20)) {\n'
        '\t\tva_l1_end = va_l1 + (128ULL << 20);\n'
        '\t}\n'
        '#endif',
        'arm_vm_init expand early IO L3 PT to 128MB',
    )

    # 2. Pre-map PCLM10 reset registers immediately after arm_vm_init & hook ALL panic stop paths
    replace_exact(
        xnu / 'osfmk/arm/arm_init.c',
        '\tarm_vm_init(xmaxmem, args);',
        '\tarm_vm_init(xmaxmem, args);\n#if defined(SWEET)\n\textern void pclm10_reset_premap(void);\n\tpclm10_reset_premap();\n#if XNU_QCOM_USB_DEBUG\n\textern void pclm10_boot_reset_premap(void);\n\tpclm10_boot_reset_premap();\n#endif\n#endif',
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

    # 5. Adapt sweet_ufs.c: SM8150 SMMU stream 0x300 bypass + opt-in SM7150 keys/touch + GPT dump/split helper
    sweet_ufs = xnu / 'bsd/dev/sweet_ufs.c'
    replace_exact(
        sweet_ufs,
        'static int\nufs_take_over_host(void)',
        'extern void pclm10_smmu_bypass_stream(uint32_t stream);\n\n'
        'static uint32_t\n'
        'pclm10_crc32(const uint8_t *buf, size_t len)\n'
        '{\n'
        '\tuint32_t crc = 0xffffffffu;\n'
        '\tfor (size_t i = 0; i < len; i++) {\n'
        '\t\tcrc ^= buf[i];\n'
        '\t\tfor (int b = 0; b < 8; b++) {\n'
        '\t\t\tcrc = (crc >> 1) ^ (0xedb88320u & (uint32_t)(-(int32_t)(crc & 1u)));\n'
        '\t\t}\n'
        '\t}\n'
        '\treturn ~crc;\n'
        '}\n\n'
        'static int\n'
        'pclm10_ufs_write_raw_lba(uint64_t lba, uint32_t count)\n'
        '{\n'
        '\tif (count == 0 || count > BOUNCE_SIZE / UFS_BLOCK || lba + count - 1 > UINT32_MAX) {\n'
        '\t\treturn EINVAL;\n'
        '\t}\n'
        '\tuint8_t cdb[10] = {\n'
        '\t\t0x2a, 0,\n'
        '\t\t(uint8_t)(lba >> 24), (uint8_t)(lba >> 16), (uint8_t)(lba >> 8), (uint8_t)lba,\n'
        '\t\t0, (uint8_t)(count >> 8), (uint8_t)count, 0\n'
        '\t};\n'
        '\treturn ufs_scsi(cdb, sizeof(cdb), count * UFS_BLOCK, true);\n'
        '}\n\n'
        'static void\n'
        'pclm10_ufs_gpt_inspect_or_split(uint8_t *hdr_buf, uint64_t entry_lba, uint32_t nent, uint32_t esz)\n'
        '{\n'
        '\tuint8_t *b = (uint8_t *)(su.dma + DMA_BOUNCE);\n'
        '\tuint32_t ufsdump = 0, split_mib = 0, ufsrw = 0;\n'
        '\t(void)PE_parse_boot_argn("ufsdump", &ufsdump, sizeof(ufsdump));\n'
        '\t(void)PE_parse_boot_argn("pclm10-split-ios", &split_mib, sizeof(split_mib));\n'
        '\t(void)PE_parse_boot_argn("ufsrw", &ufsrw, sizeof(ufsrw));\n'
        '\tif (!ufsdump && !split_mib) {\n'
        '\t\treturn;\n'
        '\t}\n'
        '\tuint64_t alt_lba = 0, first_usable = 0, last_usable = 0;\n'
        '\tuint32_t hdr_sz = 0, orig_entries_crc = 0;\n'
        '\tmemcpy(&hdr_sz, hdr_buf + 12, 4);\n'
        '\tmemcpy(&alt_lba, hdr_buf + 32, 8);\n'
        '\tmemcpy(&first_usable, hdr_buf + 40, 8);\n'
        '\tmemcpy(&last_usable, hdr_buf + 48, 8);\n'
        '\tmemcpy(&orig_entries_crc, hdr_buf + 88, 4);\n'
        '\tuint32_t calc_crc = pclm10_crc32(b, (size_t)nent * (size_t)esz);\n'
        '\tprintf("PCLM10 GPT: hdr_sz=%u nent=%u esz=%u entry_lba=0x%llx first=0x%llx last=0x%llx alt=0x%llx crc_ok=%d\\n",\n'
        '\t    hdr_sz, nent, esz, entry_lba, first_usable, last_usable, alt_lba, calc_crc == orig_entries_crc);\n'
        '\tint ud_idx = -1, ios_idx = -1, free_idx = -1;\n'
        '\tuint64_t ud_first = 0, ud_last = 0, max_end = 0;\n'
        '\tfor (uint32_t i = 0; i < nent; i++) {\n'
        '\t\tconst uint8_t *e = b + (size_t)i * (size_t)esz;\n'
        '\t\tint nonzero = 0;\n'
        '\t\tfor (int k = 0; k < 16; k++) {\n'
        '\t\t\tif (e[k] != 0) { nonzero = 1; break; }\n'
        '\t\t}\n'
        '\t\tif (!nonzero) {\n'
        '\t\t\tif (free_idx < 0) { free_idx = (int)i; }\n'
        '\t\t\tcontinue;\n'
        '\t\t}\n'
        '\t\tchar pname[37];\n'
        '\t\tunsigned c;\n'
        '\t\tfor (c = 0; c < 36; c++) {\n'
        '\t\t\tuint16_t ch = (uint16_t)(e[56 + 2 * c] | ((uint16_t)e[57 + 2 * c] << 8));\n'
        '\t\t\tif (ch == 0 || ch > 0x7f) { break; }\n'
        '\t\t\tpname[c] = (char)ch;\n'
        '\t\t}\n'
        '\t\tpname[c] = 0;\n'
        '\t\tuint64_t f = 0, l = 0;\n'
        '\t\tmemcpy(&f, e + 32, 8);\n'
        '\t\tmemcpy(&l, e + 40, 8);\n'
        '\t\tif (l > max_end) { max_end = l; }\n'
        '\t\tif (strcmp(pname, "userdata") == 0) { ud_idx = (int)i; ud_first = f; ud_last = l; }\n'
        '\t\tif (strcmp(pname, "ios") == 0) { ios_idx = (int)i; }\n'
        '\t\tif (ufsdump) {\n'
        '\t\t\tprintf("PCLM10 GPT[%2u]: %-20s 0x%08llx..0x%08llx (%llu MiB)\\n",\n'
        '\t\t\t    i, pname, f, l, (l >= f ? (l - f + 1) * UFS_BLOCK : 0) >> 20);\n'
        '\t\t}\n'
        '\t}\n'
        '\tif (!split_mib || !ufsrw) { return; }\n'
        '\tif (hdr_sz < 92 || hdr_sz > UFS_BLOCK || calc_crc != orig_entries_crc ||\n'
        '\t    entry_lba != 2 || alt_lba <= last_usable || ud_idx < 0 || free_idx < 0 ||\n'
        '\t    ios_idx >= 0 || ud_last < max_end) {\n'
        '\t\tprintf("PCLM10 GPT SPLIT REFUSED: ud=%d free=%d ios=%d max_end=0x%llx ud_last=0x%llx\\n",\n'
        '\t\t    ud_idx, free_idx, ios_idx, max_end, ud_last);\n'
        '\t\treturn;\n'
        '\t}\n'
        '\tuint64_t split_blocks = (uint64_t)split_mib * 256ULL;\n'
        '\tuint64_t min_ud_blocks = 8192ULL * 256ULL;\n'
        '\tif (ud_last <= ud_first || (ud_last - ud_first + 1) <= split_blocks + min_ud_blocks) {\n'
        '\t\tprintf("PCLM10 GPT SPLIT REFUSED: userdata too small\\n");\n'
        '\t\treturn;\n'
        '\t}\n'
        '\tuint64_t ios_first = (ud_last + 1ULL - split_blocks) & ~255ULL;\n'
        '\tif (ios_first <= ud_first + min_ud_blocks || ios_first > ud_last) {\n'
        '\t\tprintf("PCLM10 GPT SPLIT REFUSED: alignment invalid\\n");\n'
        '\t\treturn;\n'
        '\t}\n'
        '\tuint64_t new_ud_last = ios_first - 1ULL;\n'
        '\tuint8_t *ud_e = b + (size_t)ud_idx * (size_t)esz;\n'
        '\tmemcpy(ud_e + 40, &new_ud_last, 8);\n'
        '\tuint8_t *ios_e = b + (size_t)free_idx * (size_t)esz;\n'
        '\tbzero(ios_e, esz);\n'
        '\tstatic const uint8_t basic_type[16] = {\n'
        '\t\t0xa2, 0xa0, 0xd0, 0xeb, 0xe5, 0xb9, 0x33, 0x44,\n'
        '\t\t0x87, 0xc0, 0x68, 0xb6, 0xb7, 0x26, 0x99, 0xc7\n'
        '\t};\n'
        '\tstatic const uint8_t ios_uniq[16] = {\n'
        '\t\t0x10, 0x50, 0x43, 0x4c, 0x4d, 0x10, 0x49, 0x4f,\n'
        '\t\t0x53, 0x00, 0x20, 0x48, 0x33, 0x39, 0x32, 0x01\n'
        '\t};\n'
        '\tmemcpy(ios_e + 0, basic_type, 16);\n'
        '\tmemcpy(ios_e + 16, ios_uniq, 16);\n'
        '\tmemcpy(ios_e + 32, &ios_first, 8);\n'
        '\tmemcpy(ios_e + 40, &ud_last, 8);\n'
        '\tios_e[56] = \'i\'; ios_e[58] = \'o\'; ios_e[60] = \'s\';\n'
        '\tuint32_t new_crc = pclm10_crc32(b, (size_t)nent * (size_t)esz);\n'
        '\tuint32_t eblocks = (nent * esz + UFS_BLOCK - 1) / UFS_BLOCK;\n'
        '\tint werr = pclm10_ufs_write_raw_lba(alt_lba - eblocks, eblocks);\n'
        '\tif (!werr) { werr = pclm10_ufs_write_raw_lba(entry_lba, eblocks); }\n'
        '\tstatic uint8_t pri_hdr[UFS_BLOCK], alt_hdr[UFS_BLOCK];\n'
        '\tmemcpy(pri_hdr, hdr_buf, UFS_BLOCK);\n'
        '\tmemcpy(pri_hdr + 88, &new_crc, 4);\n'
        '\tbzero(pri_hdr + 16, 4);\n'
        '\tuint32_t pri_hcrc = pclm10_crc32(pri_hdr, hdr_sz);\n'
        '\tmemcpy(pri_hdr + 16, &pri_hcrc, 4);\n'
        '\tmemcpy(alt_hdr, hdr_buf, UFS_BLOCK);\n'
        '\tuint64_t one = 1ULL, alt_ent_lba = alt_lba - eblocks;\n'
        '\tmemcpy(alt_hdr + 24, &alt_lba, 8);\n'
        '\tmemcpy(alt_hdr + 32, &one, 8);\n'
        '\tmemcpy(alt_hdr + 72, &alt_ent_lba, 8);\n'
        '\tmemcpy(alt_hdr + 88, &new_crc, 4);\n'
        '\tbzero(alt_hdr + 16, 4);\n'
        '\tuint32_t alt_hcrc = pclm10_crc32(alt_hdr, hdr_sz);\n'
        '\tmemcpy(alt_hdr + 16, &alt_hcrc, 4);\n'
        '\tif (!werr) {\n'
        '\t\tmemcpy(b, alt_hdr, UFS_BLOCK);\n'
        '\t\twerr = pclm10_ufs_write_raw_lba(alt_lba, 1);\n'
        '\t}\n'
        '\tif (!werr) {\n'
        '\t\tmemcpy(b, pri_hdr, UFS_BLOCK);\n'
        '\t\twerr = pclm10_ufs_write_raw_lba(1, 1);\n'
        '\t}\n'
        '\tif (!werr) {\n'
        '\t\tbzero(b, 64u * UFS_BLOCK);\n'
        '\t\twerr = pclm10_ufs_write_raw_lba(ios_first, 64u);\n'
        '\t}\n'
        '\t(void)ufs_sync_cache();\n'
        '\t/* Restore GPT entries in bounce buffer for subsequent partition lookup */\n'
        '\t(void)ufs_read_blocks(entry_lba, eblocks);\n'
        '\tprintf("PCLM10 GPT SPLIT: rc=%d userdata=0x%llx..0x%llx (%llu MiB) ios=0x%llx..0x%llx (%llu MiB)\\n",\n'
        '\t    werr, ud_first, new_ud_last, ((new_ud_last - ud_first + 1) * UFS_BLOCK) >> 20,\n'
        '\t    ios_first, ud_last, ((ud_last - ios_first + 1) * UFS_BLOCK) >> 20);\n'
        '}\n\n'
        'static int\nufs_take_over_host(void)',
        'sweet_ufs.c SMMU declaration and GPT dump/split helpers',
    )
    replace_exact(
        sweet_ufs,
        'static int\nufs_find_partition(const char *want)',
        'static int pclm10_ufs_write_raw_lba(uint64_t lba, uint32_t count);\n'
        'static void pclm10_ufs_gpt_inspect_or_split(uint8_t *hdr_buf, uint64_t entry_lba, uint32_t nent, uint32_t esz);\n\n'
        'static int\nufs_find_partition(const char *want)',
        'sweet_ufs.c forward declarations',
    )
    replace_exact(
        sweet_ufs,
        '\tif (memcmp(b, "EFI PART", 8) != 0) {\n\t\tprintf("SWEET UFS: LUN %u has no GPT\\n", su.lun);\n\t\treturn ENXIO;\n\t}',
        '\tif (memcmp(b, "EFI PART", 8) != 0) {\n\t\tprintf("SWEET UFS: LUN %u has no GPT\\n", su.lun);\n\t\treturn ENXIO;\n\t}\n'
        '\tstatic uint8_t pclm10_gpt_hdr[UFS_BLOCK];\n'
        '\tmemcpy(pclm10_gpt_hdr, b, UFS_BLOCK);',
        'sweet_ufs.c save GPT header copy',
    )
    replace_exact(
        sweet_ufs,
        '\terror = ufs_read_blocks(entry_lba, (nent * esz + UFS_BLOCK - 1) / UFS_BLOCK);\n\tif (error) {\n\t\treturn error;\n\t}',
        '\terror = ufs_read_blocks(entry_lba, (nent * esz + UFS_BLOCK - 1) / UFS_BLOCK);\n\tif (error) {\n\t\treturn error;\n\t}\n'
        '\tpclm10_ufs_gpt_inspect_or_split(pclm10_gpt_hdr, entry_lba, nent, esz);',
        'sweet_ufs.c invoke GPT inspect/split helper',
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
        '\t/* Input drivers: enabled on PCLM10 via sweetkeys=1 / sweettouch=1 boot-args. */\n\tuint32_t enable_keys = 0, enable_touch = 0;\n\tPE_parse_boot_argn("sweetkeys", &enable_keys, sizeof(enable_keys));\n\tPE_parse_boot_argn("sweettouch", &enable_touch, sizeof(enable_touch));\n\tif (enable_keys) sweet_keys_init();\n\tif (enable_touch) sweet_touch_init();',
        'sweet_ufs.c gate PCLM10 keys/touch',
    )

    # 5b. Adapt sweet_rpmh.c, sweet_keys.c, and sweet_touch.c for PCLM10 (SM8150: PM8150 + S3706 on QUPv3 SE17)
    sweet_rpmh = xnu / 'bsd/dev/sweet_rpmh.c'
    replace_exact(
        sweet_rpmh,
        '\tif (!res || strcmp(res, "ldoc11") || mv < 2950 || mv > 3400) {\n'
        '\t\tprintf("SWEET RPMH: expected ldoc11 and 2950..3400 mV\\n");\n'
        '\t\treturn EINVAL;\n'
        '\t}',
        '\tif (!res || (strcmp(res, "ldoc11") != 0 && strcmp(res, "ldoa17") != 0) || mv < 2700 || mv > 3400) {\n'
        '\t\tprintf("SWEET RPMH: expected ldoc11/ldoa17 and 2700..3400 mV\\n");\n'
        '\t\treturn EINVAL;\n'
        '\t}',
        'sweet_rpmh.c allow ldoa17 for PCLM10',
    )

    sweet_keys = xnu / 'bsd/dev/sweet_keys.c'
    replace_exact(
        sweet_keys,
        '#define KEYS_GPIO_RT     0xc110u',
        '#define KEYS_GPIO_RT     0xc510u',
        'sweet_keys.c PM8150 GPIO6 RT_STS',
    )
    replace_exact(
        sweet_keys,
        '\tkeys_gpio_apid = keys_find_apid(0x4c1, count);\n'
        '\tuint8_t pon = 0, gpio = 0, mode = 0, en = 0, subtype = 0;\n'
        '\tbool pok = keys_read(keys_pon_apid, KEYS_PON_RT, &pon);\n'
        '\tbool gok = keys_read(keys_gpio_apid, KEYS_GPIO_RT, &gpio);\n'
        '\tbool mok = keys_read(keys_gpio_apid, 0xc140, &mode);\n'
        '\tbool eok = keys_read(keys_gpio_apid, 0xc146, &en);\n'
        '\tbool sok = keys_read(keys_gpio_apid, 0xc105, &subtype);',
        '\tkeys_gpio_apid = keys_find_apid(0x0c5, count);\n'
        '\tuint8_t pon = 0, gpio = 0, mode = 0, en = 0, subtype = 0;\n'
        '\tbool pok = keys_read(keys_pon_apid, KEYS_PON_RT, &pon);\n'
        '\tbool gok = keys_read(keys_gpio_apid, KEYS_GPIO_RT, &gpio);\n'
        '\tbool mok = keys_read(keys_gpio_apid, 0xc540, &mode);\n'
        '\tbool eok = keys_read(keys_gpio_apid, 0xc546, &en);\n'
        '\tbool sok = keys_read(keys_gpio_apid, 0xc505, &subtype);',
        'sweet_keys.c PM8150 SID0 GPIO6 (PPID 0x0c5) vol_up',
    )

    sweet_touch = xnu / 'bsd/dev/sweet_touch.c'
    replace_exact(
        sweet_touch,
        '#define TOUCH_SE_PA      0x00a84000u\n'
        '#define TOUCH_GCC_PA     0x00100000u\n'
        '#define TOUCH_NORTH_PA   0x03900000u /* DT TLMM + NORTH (0x500000) */\n'
        '#define TOUCH_GCC_VOTE   0x5200cu\n'
        '#define TOUCH_RCG       0x18148u    /* wrapper1 LOCAL S1 = global SE7 */\n'
        '#define TOUCH_GPIO_CFG(n) ((uint32_t)(n) * 0x1000u)\n'
        '#define TOUCH_GPIO_IO(n)  (TOUCH_GPIO_CFG(n) + 4u)\n'
        '#define TOUCH_MAX        10u\n'
        '#define TOUCH_GOODIX     1\n'
        '#define TOUCH_FTS        2',
        '#define TOUCH_SE_PA      0x00c80000u /* PCLM10 DT qupv3_se17_i2c (wrap2 S0) */\n'
        '#define TOUCH_GCC_PA     0x00100000u\n'
        '#define TOUCH_SOUTH_PA   0x03d00000u /* SM8150 TLMM SOUTH tile (GPIO 54/55/56/122) */\n'
        '#define TOUCH_GCC_VOTE   0x52014u    /* gcc_qupv3_wrap2 vote reg */\n'
        '#define TOUCH_RCG       0x1e148u    /* gcc_qupv3_wrap2_s0_clk_src */\n'
        '#define TOUCH_GPIO_CFG(n) ((uint32_t)(n) * 0x1000u)\n'
        '#define TOUCH_GPIO_IO(n)  (TOUCH_GPIO_CFG(n) + 4u)\n'
        '#define TOUCH_MAX        10u\n'
        '#define TOUCH_GOODIX     1\n'
        '#define TOUCH_FTS        2\n'
        '#define TOUCH_SYNAPTICS  3',
        'sweet_touch.c PCLM10 SE17 + TLMM SOUTH + GCC wrap2 S0 constants',
    )
    replace_exact(
        sweet_touch,
        '\t/* Vote on only wrapper1 and SE7, preserving other clock users. */\n'
        '\tuint32_t mask = (0x0fu << 18) | (1u << 23);\n'
        '\ttouch_wr(st.gcc, TOUCH_GCC_VOTE, touch_rd(st.gcc, TOUCH_GCC_VOTE) | mask);',
        '\t/* Vote on wrap2 s_ahb (bit 1), m_ahb (bit 2), and s0 (bit 4). */\n'
        '\tuint32_t mask = (1u << 1) | (1u << 2) | (1u << 4);\n'
        '\ttouch_wr(st.gcc, TOUCH_GCC_VOTE, touch_rd(st.gcc, TOUCH_GCC_VOTE) | mask);',
        'sweet_touch.c GCC wrap2 clock vote',
    )
    replace_exact(
        sweet_touch,
        '\tconst uint32_t branches[] = {0x18004, 0x18008, 0x1800c, 0x18010, 0x18144};',
        '\t/* Skip 0x1e008 (wrap2 s_ahb): hwcg_bit=1 keeps CLK_OFF set when idle (0x80000002). */\n'
        '\tconst uint32_t branches[] = {0x1e004, 0x1e144};',
        'sweet_touch.c GCC wrap2 branch halt regs',
    )
    replace_exact(
        sweet_touch,
        '\tint x[TOUCH_MAX], y[TOUCH_MAX];\n} st;',
        '\tint x[TOUCH_MAX], y[TOUCH_MAX];\n'
        '\tuint8_t syn_f01_data, syn_f12_data, syn_f12_d1_off, syn_f12_n, syn_f12_irq_mask;\n'
        '\tuint16_t syn_max_x, syn_max_y;\n} st;',
        'sweet_touch.c Synaptics S3706 state fields',
    )
    replace_exact(
        sweet_touch,
        'static bool\ntouch_se_init(void)',
        'static int\ntouch_write1(uint8_t addr, uint8_t reg, uint8_t val)\n{\n'
        '\tuint8_t tx[2] = {reg, val};\n'
        '\treturn touch_msg(addr, tx, sizeof(tx), NULL, 0, false);\n'
        '}\n\n'
        'static bool\ntouch_synaptics_probe(void)\n{\n'
        '\tif (touch_write1(0x20, 0xff, 0x00) != 0) {\n'
        '\t\treturn false;\n'
        '\t}\n'
        '\tuint8_t f01_q = 0, f01_c = 0, f01_d = 0;\n'
        '\tuint8_t f12_q = 0, f12_c = 0, f12_d = 0, f12_irq = 0;\n'
        '\tunsigned irq_bit = 0;\n'
        '\tfor (unsigned addr = 0xe9; addr >= 0x80; addr -= 6u) {\n'
        '\t\tuint8_t pdt[6] = {0};\n'
        '\t\tif (touch_read(0x20, (uint16_t)addr, false, pdt, sizeof(pdt)) != 0) {\n'
        '\t\t\treturn false;\n'
        '\t\t}\n'
        '\t\tif (pdt[5] == 0x00) {\n'
        '\t\t\tbreak;\n'
        '\t\t}\n'
        '\t\tunsigned nsrc = pdt[4] & 0x07u;\n'
        '\t\tif (pdt[5] == 0x01) { f01_q = pdt[0]; f01_c = pdt[2]; f01_d = pdt[3]; }\n'
        '\t\tif (pdt[5] == 0x12) {\n'
        '\t\t\tf12_q = pdt[0]; f12_c = pdt[2]; f12_d = pdt[3];\n'
        '\t\t\tif (irq_bit < 8u && nsrc) {\n'
        '\t\t\t\tf12_irq = (uint8_t)(((1u << nsrc) - 1u) << irq_bit);\n'
        '\t\t\t}\n'
        '\t\t}\n'
        '\t\tirq_bit += nsrc;\n'
        '\t}\n'
        '\tif (!f01_d || !f12_d || !f12_q) {\n'
        '\t\tprintf("SWEET TOUCH: 0x20 missing F01/F12 (f01=%#x f12=%#x)\\n", f01_d, f12_d);\n'
        '\t\treturn false;\n'
        '\t}\n'
        '\tuint8_t pid[11] = {0}, ctrl0 = 0;\n'
        '\tif (f01_q) {\n'
        '\t\t(void)touch_read(0x20, (uint16_t)(f01_q + 11u), false, pid, 10);\n'
        '\t}\n'
        '\tif (f01_c && touch_read(0x20, f01_c, false, &ctrl0, 1) == 0 && (ctrl0 & 0x03u)) {\n'
        '\t\t(void)touch_write1(0x20, f01_c, (uint8_t)((ctrl0 & ~0x07u) | 0x80u));\n'
        '\t}\n'
        '\tuint8_t d1_off = 0, d1_n = TOUCH_MAX;\n'
        '\tuint8_t sz_pres = 0, pres[16] = {0}, sbuf[64] = {0};\n'
        '\tif (touch_read(0x20, (uint16_t)(f12_q + 6u), false, &sz_pres, 1) == 0 &&\n'
        '\t    sz_pres >= 2u && sz_pres <= sizeof(pres) &&\n'
        '\t    touch_read(0x20, (uint16_t)(f12_q + 7u), false, pres, sz_pres) == 0) {\n'
        '\t\tunsigned p_off = (pres[0] == 0 && sz_pres >= 3u) ? 3u : 1u;\n'
        '\t\tunsigned s_sz = (pres[0] == 0 && sz_pres >= 3u) ? ((unsigned)pres[1] | ((unsigned)pres[2] << 8)) : pres[0];\n'
        '\t\tuint8_t pbyte = p_off < sz_pres ? pres[p_off] : 0;\n'
        '\t\tif (pbyte & 0x01u) d1_off = 1u;\n'
        '\t\tif ((pbyte & 0x02u) && s_sz > 0 && s_sz <= sizeof(sbuf) &&\n'
        '\t\t    touch_read(0x20, (uint16_t)(f12_q + 8u), false, sbuf, s_sz) == 0) {\n'
        '\t\t\tunsigned pos = 0;\n'
        '\t\t\tif ((pbyte & 0x01u) && pos < s_sz) {\n'
        '\t\t\t\tpos++;\n'
        '\t\t\t\twhile (pos < s_sz && (sbuf[pos] & 0x80u)) pos++;\n'
        '\t\t\t\tif (pos < s_sz) pos++;\n'
        '\t\t\t}\n'
        '\t\t\tif (pos < s_sz && sbuf[pos] >= 8u && sbuf[pos] <= 80u) {\n'
        '\t\t\t\td1_n = (uint8_t)(sbuf[pos] / 8u);\n'
        '\t\t\t}\n'
        '\t\t}\n'
        '\t}\n'
        '\tif (d1_n > TOUCH_MAX) d1_n = TOUCH_MAX;\n'
        '\tst.syn_f01_data = f01_d;\n'
        '\tst.syn_f12_data = f12_d;\n'
        '\tst.syn_f12_d1_off = d1_off;\n'
        '\tst.syn_f12_n = d1_n;\n'
        '\tst.syn_f12_irq_mask = f12_irq ? f12_irq : 0x04u;\n'
        '\tst.syn_max_x = 1080;\n'
        '\tst.syn_max_y = 2400;\n'
        '\t(void)f12_c;\n'
        '\tprintf("SWEET TOUCH: S3706 pid=%s f01=%#x f12=%#x d1_off=%u n=%u irq_mask=%#x\\n",\n'
        '\t    (const char *)pid, f01_d, f12_d, d1_off, d1_n, st.syn_f12_irq_mask);\n'
        '\treturn true;\n'
        '}\n\n'
        'static int\ntouch_synaptics(void)\n{\n'
        '\tuint8_t ist[2] = {0}, buf[TOUCH_MAX * 8u];\n'
        '\tint x[TOUCH_MAX] = {0}, y[TOUCH_MAX] = {0};\n'
        '\tuint32_t mask = 0;\n'
        '\tint err = touch_read(0x20, st.syn_f01_data, false, ist, 2);\n'
        '\tif (err) return err;\n'
        '\tif (!st.active && !(ist[1] & st.syn_f12_irq_mask)) {\n'
        '\t\treturn 0;\n'
        '\t}\n'
        '\tunsigned n = st.syn_f12_n ? st.syn_f12_n : TOUCH_MAX;\n'
        '\terr = touch_read(0x20, (uint16_t)(st.syn_f12_data + st.syn_f12_d1_off), false, buf, n * 8u);\n'
        '\tif (err) return err;\n'
        '\tfor (unsigned id = 0; id < n; ++id) {\n'
        '\t\tconst uint8_t *p = buf + id * 8u;\n'
        '\t\tif (p[0] == 0) continue;\n'
        '\t\tunsigned px = (unsigned)p[1] | ((unsigned)p[2] << 8);\n'
        '\t\tunsigned py = (unsigned)p[3] | ((unsigned)p[4] << 8);\n'
        '\t\tif (px > 1079) px = (px * 1079u) / (st.syn_max_x ? st.syn_max_x : 1080u);\n'
        '\t\tif (py > 2399) py = (py * 2399u) / (st.syn_max_y ? st.syn_max_y : 2400u);\n'
        '\t\tif (px > 1079 || py > 2399) continue;\n'
        '\t\tmask |= 1u << id;\n'
        '\t\tx[id] = (int)px;\n'
        '\t\ty[id] = (int)py;\n'
        '\t}\n'
        '\ttouch_report(mask, x, y);\n'
        '\treturn 0;\n'
        '}\n\n'
        'static bool\ntouch_se_init(void)',
        'sweet_touch.c add Synaptics S3706 RMI4 probe and reader',
    )
    replace_exact(
        sweet_touch,
        '\tunsigned delay_s = 45;\n'
        '\t(void)arg;\n'
        '\t(void)result;\n'
        '\t/*\n'
        '\t * Start late (boot-arg sweettouchdelay, seconds): if the bus access\n'
        '\t * below hangs the SoC, the persistent log and the GUI got there first.\n'
        '\t */\n'
        '\t(void)PE_parse_boot_argn("sweettouchdelay", &delay_s, sizeof(delay_s));\n'
        '\tprintf("SWEET TOUCH: starting in %u s\\n", delay_s);\n'
        '\ttouch_wait(delay_s * 1000u);\n'
        '\tprintf("SWEET TOUCH: mapping GCC/TLMM/SE7\\n");\n'
        '\tst.gcc = ml_io_map(TOUCH_GCC_PA, 0x60000);\n'
        '\tst.gpio = ml_io_map(TOUCH_NORTH_PA, 0x5b000);\n'
        '\tst.se = ml_io_map(TOUCH_SE_PA, 0x4000);\n'
        '\tif (!st.gcc || !st.gpio || !st.se) {\n'
        '\t\tprintf("SWEET TOUCH: MMIO mapping failed\\n");\n'
        '\t\tgoto out;\n'
        '\t}\n'
        '\tif (!touch_clocks() || !touch_se_init()) {\n'
        '\t\tgoto out;\n'
        '\t}\n'
        '\tprintf("SWEET TOUCH: TLMM reset=%#x irq=%#x vdd90=%#x; voting L11C through RPMh\\n",\n'
        '\t    touch_rd(st.gpio, TOUCH_GPIO_IO(8)), touch_rd(st.gpio, TOUCH_GPIO_IO(9)),\n'
        '\t    touch_rd(st.gpio, TOUCH_GPIO_IO(90)));\n'
        '\t/* Keep the IC in reset while its rails settle; IRQ stays an input. */\n'
        '\ttouch_gpio(9, 0, 3, 3, false, false); /* IRQ input, 8 mA, pull-up */\n'
        '\ttouch_gpio(8, 0, 3, 3, true, false);\n'
        '\tint power_error = sweet_rpmh_vreg_enable("ldoc11", 3200);\n'
        '\tif (power_error) {\n'
        '\t\tprintf("SWEET TOUCH: L11C RPMh enable failed %d; probes skipped\\n", power_error);\n'
        '\t\tgoto out;\n'
        '\t}\n'
        '\ttouch_gpio(90, 0, 0, 0, true, true); /* board digital power enable */\n'
        '\ttouch_wait(10); /* conservative rail settling; not a DT timing value */\n'
        '\ttouch_gpio(6, 1, 0, 0, false, false); /* qup11 SDA/SCL, 2 mA */\n'
        '\ttouch_gpio(7, 1, 0, 0, false, false);\n'
        '\t/* Vendor GT9896 dev_confirm: reset low 2 ms, then high. */\n'
        '\ttouch_gpio(8, 0, 3, 3, true, false);\n'
        '\ttouch_wait(2);\n'
        '\ttouch_gpio(8, 0, 3, 3, true, true);\n'
        '\ttouch_wait(300); /* exceeds vendor 5 ms test + 95 ms post-test wait */\n'
        '\tuint8_t gid[32], legacy[32], fid1 = 0, fid2 = 0;\n'
        '\tst.probing = true;\n'
        '\tint ge = touch_probe_info(GOODIX_PID_REG, gid);\n'
        '\tprintf("SWEET TOUCH: probe 0x5d err=%d pid=%02x%02x%02x%02x fw=%02x.%02x.%02x.%02x irq=%#x\\n",\n'
        '\t    ge, gid[0], gid[1], gid[2], gid[3], gid[8], gid[9], gid[10], gid[11], st.irq_status);\n'
        '\t(void)touch_probe_info(0x8240, legacy);\n'
        '\tint fe1 = touch_read(0x38, 0xa3, false, &fid1, 1);\n'
        '\tint fe2 = touch_read(0x38, 0x9f, false, &fid2, 1);\n'
        '\tprintf("SWEET TOUCH: probe 0x38 err=%d/%d chip-id=%02x:%02x irq=%#x\\n",\n'
        '\t    fe1, fe2, fid1, fid2, st.irq_status);\n'
        '\tst.probing = false;\n'
        '\tbool goodix = !ge && gid[0] == \'9\' && gid[1] == \'8\' && gid[2] == \'9\' && gid[3] == \'6\';\n'
        '\tbool fts = !fe1 && !fe2 && fid1 == 0x54 && fid2 == 0x52;\n'
        '\tif (st.dead || goodix == fts) {\n'
        '\t\tprintf("SWEET TOUCH: no unique supported IC (goodix=%d fts=%d dead=%d); stopping\\n",\n'
        '\t\t    goodix, fts, st.dead);\n'
        '\t\tgoto out;\n'
        '\t}\n'
        '\tst.kind = goodix ? TOUCH_GOODIX : TOUCH_FTS;\n'
        '\tprintf("SWEET TOUCH: selected %s, 1080x2400, poll=10ms irq-hint=%d\\n",\n'
        '\t    goodix ? "GT9896 Yellowstone" : "Focaltech 54:52", st.irq_hint);\n'
        '\tunsigned errors = 0, ticks = 0;\n'
        '\twhile (!st.dead) {\n'
        '\t\t/* Optional IRQ hint still reads every 100ms, and always while down. */\n'
        '\t\tif (!st.irq_hint || st.active || !(touch_rd(st.gpio, TOUCH_GPIO_IO(9)) & 1u) ||\n'
        '\t\t    (++ticks % 10u) == 0) {\n'
        '\t\t\tint err = st.kind == TOUCH_GOODIX ? touch_goodix() : touch_focaltech();',
        '\tunsigned delay_s = 15;\n'
        '\t(void)arg;\n'
        '\t(void)result;\n'
        '\t(void)PE_parse_boot_argn("sweettouchdelay", &delay_s, sizeof(delay_s));\n'
        '\tprintf("SWEET TOUCH: starting in %u s (PCLM10 SE17 + S3706)\\n", delay_s);\n'
        '\ttouch_wait(delay_s * 1000u);\n'
        '\tprintf("SWEET TOUCH: mapping GCC/TLMM_SOUTH/SE17\\n");\n'
        '\tst.gcc = ml_io_map(TOUCH_GCC_PA, 0x60000);\n'
        '\tst.gpio = ml_io_map(TOUCH_SOUTH_PA, 0x80000);\n'
        '\tst.se = ml_io_map(TOUCH_SE_PA, 0x4000);\n'
        '\tif (!st.gcc || !st.gpio || !st.se) {\n'
        '\t\tprintf("SWEET TOUCH: MMIO mapping failed\\n");\n'
        '\t\tgoto out;\n'
        '\t}\n'
        '\tif (!touch_clocks() || !touch_se_init()) {\n'
        '\t\tgoto out;\n'
        '\t}\n'
        '\tprintf("SWEET TOUCH: TLMM reset54=%#x irq122=%#x; voting ldoa17 through RPMh\\n",\n'
        '\t    touch_rd(st.gpio, TOUCH_GPIO_IO(54)), touch_rd(st.gpio, TOUCH_GPIO_IO(122)));\n'
        '\ttouch_gpio(122, 0, 3, 3, false, false); /* IRQ input, 8 mA, pull-up */\n'
        '\ttouch_gpio(54, 0, 3, 3, true, false);\n'
        '\tint power_error = sweet_rpmh_vreg_enable("ldoa17", 3000);\n'
        '\tif (power_error) {\n'
        '\t\tprintf("SWEET TOUCH: ldoa17 RPMh rc=%d (continuing with ABL rail)\\n", power_error);\n'
        '\t}\n'
        '\ttouch_wait(10);\n'
        '\ttouch_gpio(55, 1, 0, 0, false, false); /* qup17 SDA/SCL, 2 mA */\n'
        '\ttouch_gpio(56, 1, 0, 0, false, false);\n'
        '\ttouch_gpio(54, 0, 3, 3, true, false);\n'
        '\ttouch_wait(2);\n'
        '\ttouch_gpio(54, 0, 3, 3, true, true);\n'
        '\ttouch_wait(150);\n'
        '\tst.probing = true;\n'
        '\tbool syn = touch_synaptics_probe();\n'
        '\tbool goodix = false, fts = false;\n'
        '\tif (!syn) {\n'
        '\t\tuint8_t gid[32] = {0}, fid1 = 0, fid2 = 0;\n'
        '\t\tint ge = touch_probe_info(GOODIX_PID_REG, gid);\n'
        '\t\tint fe1 = touch_read(0x38, 0xa3, false, &fid1, 1);\n'
        '\t\tint fe2 = touch_read(0x38, 0x9f, false, &fid2, 1);\n'
        '\t\tgoodix = !ge && gid[0] == \'9\' && gid[1] == \'8\' && gid[2] == \'9\' && gid[3] == \'6\';\n'
        '\t\tfts = !fe1 && !fe2 && fid1 == 0x54 && fid2 == 0x52;\n'
        '\t}\n'
        '\tst.probing = false;\n'
        '\tif (st.dead || (!syn && goodix == fts)) {\n'
        '\t\tprintf("SWEET TOUCH: no supported IC (syn=%d goodix=%d fts=%d dead=%d irq=%#x); stopping\\n",\n'
        '\t\t    syn, goodix, fts, st.dead, st.irq_status);\n'
        '\t\tgoto out;\n'
        '\t}\n'
        '\tst.kind = syn ? TOUCH_SYNAPTICS : (goodix ? TOUCH_GOODIX : TOUCH_FTS);\n'
        '\tprintf("SWEET TOUCH: selected %s, 1080x2400, poll=10ms irq-hint=%d\\n",\n'
        '\t    syn ? "Synaptics S3706" : (goodix ? "GT9896" : "Focaltech"), st.irq_hint);\n'
        '\tunsigned errors = 0, ticks = 0;\n'
        '\twhile (!st.dead) {\n'
        '\t\tif (!st.irq_hint || st.active || !(touch_rd(st.gpio, TOUCH_GPIO_IO(122)) & 1u) ||\n'
        '\t\t    (++ticks % 10u) == 0) {\n'
        '\t\t\tint err = st.kind == TOUCH_SYNAPTICS ? touch_synaptics() : (st.kind == TOUCH_GOODIX ? touch_goodix() : touch_focaltech());',
        'sweet_touch.c wire PCLM10 TLMM SOUTH/SE17/S3706 thread',
    )
    replace_exact(
        sweet_touch,
        'static int\n'
        'touch_msg(uint8_t addr, const uint8_t *tx, unsigned nt, uint8_t *rx,\n'
        '    unsigned nr, bool stretch)\n'
        '{\n'
        '\tst.irq_status = st.irq_last = 0;\n'
        '\tst.sent = st.got = 0;\n'
        '\tint err = touch_msg_run(addr, tx, nt, rx, nr, stretch);',
        '/* TLMM SOUTH GPIO 55 (SDA) / GPIO 56 (SCL) bit-bang fallback when SE17 FIFO is TZ-disabled. */\n'
        'static bool touch_gpio_i2c;\n'
        'static bool touch_bb_inited;\n'
        'static bool touch_bb_held;\n'
        '\n'
        'static bool\n'
        'touch_bb_spmi_xfer(uint16_t apid, uint16_t addr, bool write, uint8_t *val)\n'
        '{\n'
        '\tvm_offset_t obs = ml_io_map(0x0e600000u, 0x10000u);\n'
        '\tvm_offset_t ch = write ? ml_io_map(0x0c600000u + 0x10000u * apid, 0x1000u) : 0;\n'
        '\tif (!obs || (write && !ch)) {\n'
        '\t\treturn false;\n'
        '\t}\n'
        '\tvm_offset_t base = write ? ch : (obs + 0x80u * apid);\n'
        '\tif (write) {\n'
        '\t\t__asm__ volatile("dmb oshst" ::: "memory");\n'
        '\t\t*(volatile uint32_t *)(base + 0x10u) = *val;\n'
        '\t\t__asm__ volatile("dsb osh" ::: "memory");\n'
        '\t}\n'
        '\tuint32_t op = write ? 0u : 1u; /* EXT_WRITEL=0, EXT_READL=1 */\n'
        '\tuint32_t cmd = (op << 27) | ((uint32_t)(addr & 0xffu) << 4);\n'
        '\t__asm__ volatile("dmb oshst" ::: "memory");\n'
        '\t*(volatile uint32_t *)(base + 0x00u) = cmd;\n'
        '\t__asm__ volatile("dsb osh" ::: "memory");\n'
        '\tfor (unsigned i = 0; i < 1000u; ++i) {\n'
        '\t\tuint32_t stv = *(volatile uint32_t *)(base + 0x08u);\n'
        '\t\t__asm__ volatile("dmb oshld" ::: "memory");\n'
        '\t\tif (stv & 1u) {\n'
        '\t\t\tif (stv & 0x0eu) {\n'
        '\t\t\t\treturn false;\n'
        '\t\t\t}\n'
        '\t\t\tif (!write) {\n'
        '\t\t\t\t*val = (uint8_t)(*(volatile uint32_t *)(base + 0x18u));\n'
        '\t\t\t\t__asm__ volatile("dmb oshld" ::: "memory");\n'
        '\t\t\t}\n'
        '\t\t\treturn true;\n'
        '\t\t}\n'
        '\t\tdelay(1);\n'
        '\t}\n'
        '\treturn false;\n'
        '}\n'
        '\n'
        'static void\n'
        'touch_bb_pin_dir(unsigned pin, bool drive_low)\n'
        '{\n'
        '\tuint32_t off = TOUCH_GPIO_CFG(pin);\n'
        '\tuint32_t cfg = touch_rd(st.gpio, off);\n'
        '\tif (drive_low) {\n'
        '\t\ttouch_wr(st.gpio, TOUCH_GPIO_IO(pin),\n'
        '\t\t    touch_rd(st.gpio, TOUCH_GPIO_IO(pin)) & ~2u);\n'
        '\t\tcfg = (cfg & ~0x3ffu) | (1u << 6) | 0x200u;\n'
        '\t} else {\n'
        '\t\tcfg = (cfg & ~0x3ffu) | (1u << 6);\n'
        '\t}\n'
        '\ttouch_wr(st.gpio, off, cfg);\n'
        '}\n'
        '\n'
        'static bool\n'
        'touch_bb_sda_rd(void)\n'
        '{\n'
        '\treturn (touch_rd(st.gpio, TOUCH_GPIO_IO(55u)) & 1u) != 0;\n'
        '}\n'
        '\n'
        'static bool\n'
        'touch_bb_scl_rd(void)\n'
        '{\n'
        '\treturn (touch_rd(st.gpio, TOUCH_GPIO_IO(56u)) & 1u) != 0;\n'
        '}\n'
        '\n'
        'static bool\n'
        'touch_bb_scl_hi(void)\n'
        '{\n'
        '\ttouch_bb_pin_dir(56u, false);\n'
        '\tfor (unsigned i = 0; i < 500u; ++i) {\n'
        '\t\tif (touch_bb_scl_rd()) {\n'
        '\t\t\tdelay(4);\n'
        '\t\t\treturn true;\n'
        '\t\t}\n'
        '\t\tdelay(1);\n'
        '\t}\n'
        '\treturn false;\n'
        '}\n'
        '\n'
        'static void\n'
        'touch_bb_stop(void)\n'
        '{\n'
        '\ttouch_bb_pin_dir(55u, true);\n'
        '\tdelay(3);\n'
        '\t(void)touch_bb_scl_hi();\n'
        '\ttouch_bb_pin_dir(55u, false);\n'
        '\tdelay(4);\n'
        '}\n'
        '\n'
        'static void\n'
        'touch_bb_recover(void)\n'
        '{\n'
        '\ttouch_bb_pin_dir(55u, false);\n'
        '\tfor (unsigned i = 0; i < 9u && !touch_bb_sda_rd(); ++i) {\n'
        '\t\ttouch_bb_pin_dir(56u, true);\n'
        '\t\tdelay(4);\n'
        '\t\t(void)touch_bb_scl_hi();\n'
        '\t}\n'
        '\ttouch_bb_stop();\n'
        '}\n'
        '\n'
        'static bool\n'
        'touch_bb_tx_byte(uint8_t b)\n'
        '{\n'
        '\tfor (int bit = 7; bit >= 0; --bit) {\n'
        '\t\ttouch_bb_pin_dir(55u, ((b >> bit) & 1u) == 0);\n'
        '\t\tdelay(3);\n'
        '\t\tif (!touch_bb_scl_hi()) {\n'
        '\t\t\treturn false;\n'
        '\t\t}\n'
        '\t\ttouch_bb_pin_dir(56u, true);\n'
        '\t\tdelay(2);\n'
        '\t}\n'
        '\ttouch_bb_pin_dir(55u, false);\n'
        '\tdelay(3);\n'
        '\tif (!touch_bb_scl_hi()) {\n'
        '\t\treturn false;\n'
        '\t}\n'
        '\tbool ack = !touch_bb_sda_rd();\n'
        '\ttouch_bb_pin_dir(56u, true);\n'
        '\tdelay(2);\n'
        '\treturn ack;\n'
        '}\n'
        '\n'
        'static bool\n'
        'touch_bb_rx_byte(bool send_ack, uint8_t *out)\n'
        '{\n'
        '\tuint8_t val = 0;\n'
        '\ttouch_bb_pin_dir(55u, false);\n'
        '\tfor (int bit = 7; bit >= 0; --bit) {\n'
        '\t\tdelay(3);\n'
        '\t\tif (!touch_bb_scl_hi()) {\n'
        '\t\t\treturn false;\n'
        '\t\t}\n'
        '\t\tif (touch_bb_sda_rd()) {\n'
        '\t\t\tval |= (uint8_t)(1u << bit);\n'
        '\t\t}\n'
        '\t\ttouch_bb_pin_dir(56u, true);\n'
        '\t\tdelay(2);\n'
        '\t}\n'
        '\ttouch_bb_pin_dir(55u, send_ack);\n'
        '\tdelay(3);\n'
        '\tif (!touch_bb_scl_hi()) {\n'
        '\t\treturn false;\n'
        '\t}\n'
        '\ttouch_bb_pin_dir(56u, true);\n'
        '\tdelay(2);\n'
        '\ttouch_bb_pin_dir(55u, false);\n'
        '\t*out = val;\n'
        '\treturn true;\n'
        '}\n'
        '\n'
        'static void\n'
        'touch_bb_ensure_rails(void)\n'
        '{\n'
        '\ttouch_bb_inited = true;\n'
        '\ttouch_bb_pin_dir(55u, false);\n'
        '\ttouch_bb_pin_dir(56u, false);\n'
        '\tdelay(50);\n'
        '\tbool sda = touch_bb_sda_rd(), scl = touch_bb_scl_rd();\n'
        '\tprintf("SWEET TOUCH: bb-i2c init sda=%d scl=%d\\n", sda, scl);\n'
        '\tif (sda && scl) {\n'
        '\t\treturn;\n'
        '\t}\n'
        '\tvm_offset_t core = ml_io_map(0x0c440000u, 0x2000u);\n'
        '\tvm_offset_t cnfg = ml_io_map(0x0c40a000u, 0x1000u);\n'
        '\tif (!core || !cnfg) {\n'
        '\t\treturn;\n'
        '\t}\n'
        '\tuint16_t apid = 0xffffu;\n'
        '\tfor (uint32_t i = 0; i < 512u; ++i) {\n'
        '\t\tuint32_t m = *(volatile uint32_t *)(core + 0x900u + 4u * i);\n'
        '\t\t__asm__ volatile("dmb oshld" ::: "memory");\n'
        '\t\tif (m != 0 && ((m >> 8) & 0xfffu) == 0x4c4u) {\n'
        '\t\t\tuint32_t own = *(volatile uint32_t *)(cnfg + 0x700u + 4u * i) & 7u;\n'
        '\t\t\tif (apid == 0xffffu || own == 0u) {\n'
        '\t\t\t\tapid = (uint16_t)i;\n'
        '\t\t\t}\n'
        '\t\t}\n'
        '\t}\n'
        '\tuint8_t mode = 0, out = 0, en = 0;\n'
        '\tbool r1 = apid != 0xffffu && touch_bb_spmi_xfer(apid, 0xc440u, false, &mode);\n'
        '\tbool r2 = apid != 0xffffu && touch_bb_spmi_xfer(apid, 0xc444u, false, &out);\n'
        '\tbool r3 = apid != 0xffffu && touch_bb_spmi_xfer(apid, 0xc446u, false, &en);\n'
        '\tprintf("SWEET TOUCH: pm8150l gpio5 apid=%u mode=%#x/%d out=%#x/%d en=%#x/%d\\n",\n'
        '\t    apid, mode, r1, out, r2, en, r3);\n'
        '\tif (apid != 0xffffu && (!sda || !scl)) {\n'
        '\t\tuint8_t v_mode = 1u, v_low = 0u, v_hi = 0x80u, v_en = 0x80u;\n'
        '\t\t(void)touch_bb_spmi_xfer(apid, 0xc440u, true, &v_mode);\n'
        '\t\t(void)touch_bb_spmi_xfer(apid, 0xc444u, true, &v_low);\n'
        '\t\t(void)touch_bb_spmi_xfer(apid, 0xc446u, true, &v_en);\n'
        '\t\ttouch_wait(10);\n'
        '\t\tsda = touch_bb_sda_rd(); scl = touch_bb_scl_rd();\n'
        '\t\tprintf("SWEET TOUCH: pm8150l gpio5 low -> sda=%d scl=%d\\n", sda, scl);\n'
        '\t\tif (!sda || !scl) {\n'
        '\t\t\t(void)touch_bb_spmi_xfer(apid, 0xc444u, true, &v_hi);\n'
        '\t\t\ttouch_wait(10);\n'
        '\t\t\tsda = touch_bb_sda_rd(); scl = touch_bb_scl_rd();\n'
        '\t\t\tprintf("SWEET TOUCH: pm8150l gpio5 high -> sda=%d scl=%d\\n", sda, scl);\n'
        '\t\t}\n'
        '\t\tif (sda && scl) {\n'
        '\t\t\t/* 1.8V came up late; re-pulse S3706 reset with all rails live. */\n'
        '\t\t\ttouch_gpio(54, 0, 3, 3, true, false);\n'
        '\t\t\ttouch_wait(5);\n'
        '\t\t\ttouch_gpio(54, 0, 3, 3, true, true);\n'
        '\t\t\ttouch_wait(150);\n'
        '\t\t}\n'
        '\t}\n'
        '}\n'
        '\n'
        'static int\n'
        'touch_bb_msg(uint8_t addr, const uint8_t *tx, unsigned nt, uint8_t *rx,\n'
        '    unsigned nr, bool stretch)\n'
        '{\n'
        '\tif (st.dead || (nt && nr) || (!nt && !nr)) {\n'
        '\t\treturn EIO;\n'
        '\t}\n'
        '\tif (!touch_bb_inited) {\n'
        '\t\ttouch_bb_ensure_rails();\n'
        '\t}\n'
        '\tif (!touch_bb_held) {\n'
        '\t\ttouch_bb_pin_dir(55u, false);\n'
        '\t\ttouch_bb_pin_dir(56u, false);\n'
        '\t\tdelay(4);\n'
        '\t\tif (!touch_bb_sda_rd() || !touch_bb_scl_rd()) {\n'
        '\t\t\ttouch_bb_recover();\n'
        '\t\t}\n'
        '\t} else {\n'
        '\t\ttouch_bb_pin_dir(55u, false);\n'
        '\t\tdelay(3);\n'
        '\t\tif (!touch_bb_scl_hi()) {\n'
        '\t\t\ttouch_bb_held = false;\n'
        '\t\t\ttouch_bb_stop();\n'
        '\t\t\treturn EIO;\n'
        '\t\t}\n'
        '\t}\n'
        '\ttouch_bb_pin_dir(55u, true);\n'
        '\tdelay(4);\n'
        '\ttouch_bb_pin_dir(56u, true);\n'
        '\tdelay(2);\n'
        '\tuint8_t ab = (uint8_t)((addr << 1) | (nr ? 1u : 0u));\n'
        '\tif (!touch_bb_tx_byte(ab)) {\n'
        '\t\ttouch_bb_held = false;\n'
        '\t\ttouch_bb_stop();\n'
        '\t\treturn ENXIO;\n'
        '\t}\n'
        '\tfor (unsigned i = 0; i < nt; ++i) {\n'
        '\t\tif (!touch_bb_tx_byte(tx[i])) {\n'
        '\t\t\ttouch_bb_held = false;\n'
        '\t\t\ttouch_bb_stop();\n'
        '\t\t\treturn EIO;\n'
        '\t\t}\n'
        '\t\tst.sent = i + 1u;\n'
        '\t}\n'
        '\tfor (unsigned i = 0; i < nr; ++i) {\n'
        '\t\tif (!touch_bb_rx_byte(i + 1u < nr, &rx[i])) {\n'
        '\t\t\ttouch_bb_held = false;\n'
        '\t\t\ttouch_bb_stop();\n'
        '\t\t\treturn EIO;\n'
        '\t\t}\n'
        '\t\tst.got = i + 1u;\n'
        '\t}\n'
        '\tif (stretch && !nr) {\n'
        '\t\ttouch_bb_held = true;\n'
        '\t\treturn 0;\n'
        '\t}\n'
        '\ttouch_bb_stop();\n'
        '\ttouch_bb_held = false;\n'
        '\treturn 0;\n'
        '}\n'
        '\n'
        'static int\n'
        'touch_msg(uint8_t addr, const uint8_t *tx, unsigned nt, uint8_t *rx,\n'
        '    unsigned nr, bool stretch)\n'
        '{\n'
        '\tst.irq_status = st.irq_last = 0;\n'
        '\tst.sent = st.got = 0;\n'
        '\tint err = touch_gpio_i2c ? touch_bb_msg(addr, tx, nt, rx, nr, stretch) :\n'
        '\t    touch_msg_run(addr, tx, nt, rx, nr, stretch);',
        'sweet_touch.c add TLMM GPIO 55/56 I2C bitbang fallback',
    )
    replace_exact(
        sweet_touch,
        '\tif (((fw >> 8) & 0xffu) != 3u || (touch_rd(st.se, 0x64) & 1u) ||\n'
        '\t    ((tx >> 24) & 0x3fu) != 32u || ((rx >> 24) & 0x3fu) != 32u ||\n'
        '\t    ((tx >> 16) & 0x3fu) < 2u || ((rx >> 16) & 0x3fu) < 2u) {\n'
        '\t\tprintf("SWEET TOUCH: need ABL-loaded I2C firmware and 32-bit FIFOs; probes skipped\\n");\n'
        '\t\treturn false;\n'
        '\t}',
        '\tif ((touch_rd(st.se, 0x64) & 1u) != 0u) {\n'
        '\t\ttouch_gpio_i2c = true;\n'
        '\t\tprintf("SWEET TOUCH: SE17 FIFO disabled (0x64=%#x); using TLMM GPIO 55/56 I2C bitbang\\n",\n'
        '\t\t    touch_rd(st.se, 0x64));\n'
        '\t\treturn true;\n'
        '\t}\n'
        '\tif (((fw >> 8) & 0xffu) != 3u ||\n'
        '\t    ((tx >> 24) & 0x3fu) != 32u || ((rx >> 24) & 0x3fu) != 32u ||\n'
        '\t    ((tx >> 16) & 0x3fu) < 2u || ((rx >> 16) & 0x3fu) < 2u) {\n'
        '\t\tprintf("SWEET TOUCH: need ABL-loaded I2C firmware and 32-bit FIFOs; probes skipped\\n");\n'
        '\t\treturn false;\n'
        '\t}',
        'sweet_touch.c select GPIO bitbang when SE17 FIFO_IF_DISABLE is set',
    )
    replace_exact(
        sweet_touch,
        '\ttouch_gpio(55, 1, 0, 0, false, false); /* qup17 SDA/SCL, 2 mA */\n'
        '\ttouch_gpio(56, 1, 0, 0, false, false);',
        '\tif (!touch_gpio_i2c) {\n'
        '\t\ttouch_gpio(55, 1, 0, 0, false, false); /* qup17 SDA/SCL, 2 mA */\n'
        '\t\ttouch_gpio(56, 1, 0, 0, false, false);\n'
        '\t}',
        'sweet_touch.c keep GPIO 55/56 out of qup17 mux in bitbang mode',
    )
    replace_exact(
        sweet_touch,
        '\tst.syn_f01_data = f01_d;\n'
        '\tst.syn_f12_data = f12_d;\n'
        '\tst.syn_f12_d1_off = d1_off;\n'
        '\tst.syn_f12_n = d1_n;\n'
        '\tst.syn_f12_irq_mask = f12_irq ? f12_irq : 0x04u;\n'
        '\tst.syn_max_x = 1080;\n'
        '\tst.syn_max_y = 2400;\n'
        '\t(void)f12_c;\n'
        '\tprintf("SWEET TOUCH: S3706 pid=%s f01=%#x f12=%#x d1_off=%u n=%u irq_mask=%#x\\n",\n'
        '\t    (const char *)pid, f01_d, f12_d, d1_off, d1_n, st.syn_f12_irq_mask);\n'
        '\treturn true;\n'
        '}\n'
        '\n'
        'static int\n'
        'touch_synaptics(void)\n'
        '{\n'
        '\tuint8_t ist[2] = {0}, buf[TOUCH_MAX * 8u];\n'
        '\tint x[TOUCH_MAX] = {0}, y[TOUCH_MAX] = {0};\n'
        '\tuint32_t mask = 0;\n'
        '\tint err = touch_read(0x20, st.syn_f01_data, false, ist, 2);\n'
        '\tif (err) return err;\n'
        '\tif (!st.active && !(ist[1] & st.syn_f12_irq_mask)) {\n'
        '\t\treturn 0;\n'
        '\t}\n'
        '\tunsigned n = st.syn_f12_n ? st.syn_f12_n : TOUCH_MAX;\n'
        '\terr = touch_read(0x20, (uint16_t)(st.syn_f12_data + st.syn_f12_d1_off), false, buf, n * 8u);\n'
        '\tif (err) return err;\n'
        '\tfor (unsigned id = 0; id < n; ++id) {\n'
        '\t\tconst uint8_t *p = buf + id * 8u;\n'
        '\t\tif (p[0] == 0) continue;\n'
        '\t\tunsigned px = (unsigned)p[1] | ((unsigned)p[2] << 8);\n'
        '\t\tunsigned py = (unsigned)p[3] | ((unsigned)p[4] << 8);\n'
        '\t\tif (px > 1079) px = (px * 1079u) / (st.syn_max_x ? st.syn_max_x : 1080u);\n'
        '\t\tif (py > 2399) py = (py * 2399u) / (st.syn_max_y ? st.syn_max_y : 2400u);\n'
        '\t\tif (px > 1079 || py > 2399) continue;\n'
        '\t\tmask |= 1u << id;\n'
        '\t\tx[id] = (int)px;\n'
        '\t\ty[id] = (int)py;\n'
        '\t}\n'
        '\ttouch_report(mask, x, y);\n'
        '\treturn 0;\n'
        '}',
        '\tst.syn_f01_data = f01_d;\n'
        '\tst.syn_f12_data = f12_d;\n'
        '\t/* OPPO S3706 stores Data1 directly at F12_2D_DATA_BASE (+0), Data15 at +3. */\n'
        '\tst.syn_f12_d1_off = 0;\n'
        '\tst.syn_f12_n = d1_n;\n'
        '\tst.syn_f12_irq_mask = f12_irq ? f12_irq : 0x04u;\n'
        '\tst.syn_max_x = 1080;\n'
        '\tst.syn_max_y = 2400;\n'
        '\t(void)f12_c;\n'
        '\t(void)d1_off;\n'
        '\tif (f01_c) {\n'
        '\t\tuint8_t c0 = 0, c1 = 0, dummy[2] = {0};\n'
        '\t\t(void)touch_read(0x20, f01_c, false, &c0, 1);\n'
        '\t\t(void)touch_read(0x20, (uint8_t)(f01_c + 1u), false, &c1, 1);\n'
        '\t\tuint8_t nc0 = (uint8_t)((c0 & 0xf8u) | 0x80u), nc1 = 0x7fu;\n'
        '\t\tconst uint8_t w0[2] = {f01_c, nc0};\n'
        '\t\tconst uint8_t w1[2] = {(uint8_t)(f01_c + 1u), nc1};\n'
        '\t\t(void)touch_msg(0x20, w0, 2, NULL, 0, false);\n'
        '\t\t(void)touch_msg(0x20, w1, 2, NULL, 0, false);\n'
        '\t\t(void)touch_read(0x20, f01_d, false, dummy, 2);\n'
        '\t\tprintf("SWEET TOUCH: S3706 f01_c=%#x ctrl=%#x->%#x irq=%#x->%#x ist=%#x/%#x\\n",\n'
        '\t\t    f01_c, c0, nc0, c1, nc1, dummy[0], dummy[1]);\n'
        '\t}\n'
        '\tuint8_t att0[2] = {0}, d0_samp[16] = {0};\n'
        '\t(void)touch_read(0x20, (uint16_t)(f12_d + 3u), false, att0, 2);\n'
        '\t(void)touch_read(0x20, f12_d, false, d0_samp, 16);\n'
        '\tprintf("SWEET TOUCH: S3706 pid=%s f01=%#x f12=%#x d1_off=%u n=%u irq_mask=%#x att=%02x%02x d0=%02x%02x%02x%02x%02x%02x\\n",\n'
        '\t    (const char *)pid, f01_d, f12_d, st.syn_f12_d1_off, d1_n, st.syn_f12_irq_mask,\n'
        '\t    att0[1], att0[0], d0_samp[0], d0_samp[1], d0_samp[2], d0_samp[3], d0_samp[4], d0_samp[5]);\n'
        '\treturn true;\n'
        '}\n'
        '\n'
        'static int\n'
        'touch_synaptics(void)\n'
        '{\n'
        '\tuint8_t ist[2] = {0}, attb[2] = {0}, buf[TOUCH_MAX * 8u];\n'
        '\tint x[TOUCH_MAX] = {0}, y[TOUCH_MAX] = {0};\n'
        '\tuint32_t mask = 0;\n'
        '\tint err = touch_read(0x20, st.syn_f01_data, false, ist, 2);\n'
        '\tif (err) return err;\n'
        '\tif (!st.active && !(ist[1] & st.syn_f12_irq_mask)) {\n'
        '\t\treturn 0;\n'
        '\t}\n'
        '\terr = touch_read(0x20, (uint16_t)(st.syn_f12_data + 3u), false, attb, 2);\n'
        '\tif (err) return err;\n'
        '\tuint16_t att = (uint16_t)attb[0] | ((uint16_t)attb[1] << 8);\n'
        '\tunsigned n = st.syn_f12_n ? st.syn_f12_n : TOUCH_MAX;\n'
        '\tunsigned active_hi = 0;\n'
        '\tfor (unsigned id = 0; id < n; ++id) {\n'
        '\t\tif (att & (1u << id)) active_hi = id + 1u;\n'
        '\t}\n'
        '\tif (active_hi == 0u) {\n'
        '\t\tif (st.active) touch_report(0, x, y);\n'
        '\t\treturn 0;\n'
        '\t}\n'
        '\terr = touch_read(0x20, (uint16_t)(st.syn_f12_data + st.syn_f12_d1_off), false, buf, active_hi * 8u);\n'
        '\tif (err) return err;\n'
        '\tfor (unsigned id = 0; id < active_hi; ++id) {\n'
        '\t\tif (!(att & (1u << id))) continue;\n'
        '\t\tconst uint8_t *p = buf + id * 8u;\n'
        '\t\tif (p[0] == 0) continue;\n'
        '\t\tunsigned px = (unsigned)p[1] | ((unsigned)(p[2] & 0x0fu) << 8);\n'
        '\t\tunsigned py = (unsigned)p[3] | ((unsigned)(p[4] & 0x0fu) << 8);\n'
        '\t\tif (px > 1079) px = (px * 1079u) / (st.syn_max_x ? st.syn_max_x : 1080u);\n'
        '\t\tif (py > 2399) py = (py * 2399u) / (st.syn_max_y ? st.syn_max_y : 2400u);\n'
        '\t\tif (px > 1079 || py > 2399) continue;\n'
        '\t\tmask |= 1u << id;\n'
        '\t\tx[id] = (int)px;\n'
        '\t\ty[id] = (int)py;\n'
        '\t}\n'
        '\ttouch_report(mask, x, y);\n'
        '\treturn 0;\n'
        '}',
        'sweet_touch.c fix S3706 F12_DATA_BASE (+0), DATA15 obj_attention, and F01 normal/IRQ init',
    )

    # 6. Start USB ECM + watchdog (when not using QUD) and run optional UFS GPT dump/split before md0
    replace_exact(
        xnu / 'iokit/bsddev/IOKitBSDInit.cpp',
        '\tif (strcmp(rdBootVar, "ufsroot0") == 0) {',
        '\t{\n'
        '\t\tuint32_t qud_enabled = 0, ufsdump = 0, split_mib = 0;\n'
        '\t\tPE_parse_boot_argn("qcom-usb-debug", &qud_enabled, sizeof(qud_enabled));\n'
        '\t\tif (qud_enabled != 1) {\n'
        '\t\t\textern void pclm10_watchdog_start(void);\n'
        '\t\t\textern void sweet_usb_probe(void);\n'
        '\t\t\tpclm10_watchdog_start();\n'
        '\t\t\tsweet_usb_probe();\n'
        '\t\t}\n'
        '\t\tPE_parse_boot_argn("ufsdump", &ufsdump, sizeof(ufsdump));\n'
        '\t\tPE_parse_boot_argn("pclm10-split-ios", &split_mib, sizeof(split_mib));\n'
        '\t\tif ((ufsdump || split_mib) && strcmp(rdBootVar, "ufsroot0") != 0) {\n'
        '\t\t\textern int sweet_ufs_rootdev(dev_t *dev);\n'
        '\t\t\tdev_t dummy_ufs = 0;\n'
        '\t\t\t(void)sweet_ufs_rootdev(&dummy_ufs);\n'
        '\t\t}\n'
        '\t}\n'
        '\tif (strcmp(rdBootVar, "ufsroot0") == 0) {',
        'IOKitBSDInit start USB, watchdog, and optional UFS dump/split',
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


def apply_qud_overlay(xnu: Path, qud_dir: Path, boot_reset_c: Path | None) -> None:
    dst = xnu / 'pexpert/qcom_usb_debug'
    if dst.exists():
        shutil.rmtree(dst)
    dst.mkdir(parents=True)
    for sub in ('include', 'kernel', 'soc', 'usb'):
        shutil.copytree(qud_dir / sub, dst / sub)
    if boot_reset_c is None:
        raise SystemExit('--boot-reset-c is required when --qud-dir is passed')
    shutil.copy2(boot_reset_c, dst / 'Pclm10BootReset.c')

    replace_exact(
        dst / 'include/QcomUsbSoc.h',
        '#ifndef QUD_SOC_HEADER\n#error "Select hardware with -DQUD_SOC_HEADER=\\"soc/Sm8550.h\\""\n#endif',
        '#ifndef QUD_SOC_HEADER\n#define QUD_SOC_HEADER "soc/Sm8150.h"\n#endif',
        'QcomUsbSoc.h default Sm8150.h',
    )

    replace_exact(
        xnu / 'makedefs/MakeInc.def',
        'MACHINE_FLAGS_ARM64_SWEET = -DARM64_BOARD_CONFIG_SWEET                -march=armv8.2-a+crypto',
        'MACHINE_FLAGS_ARM64_SWEET = -DARM64_BOARD_CONFIG_SWEET                -march=armv8.2-a+crypto '
        '-DXNU_QCOM_USB_DEBUG=1 -I$(SRCROOT)/pexpert/qcom_usb_debug/include '
        '-I$(SRCROOT)/pexpert/qcom_usb_debug -I$(SRCROOT)/pexpert/qcom_usb_debug/kernel',
        'MakeInc.def XNU_QCOM_USB_DEBUG flags',
    )

    replace_exact(
        xnu / 'pexpert/conf/files',
        'pexpert/gen/kcformat.c\t\t\t\tstandard',
        'pexpert/gen/kcformat.c\t\t\t\tstandard\n'
        'pexpert/qcom_usb_debug/kernel/QcomXnuUsbDebug.c\t\tstandard\n'
        'pexpert/qcom_usb_debug/kernel/QcomUsbScreenshot.c\tstandard\n'
        'pexpert/qcom_usb_debug/usb/Dwc3.c\t\t\t\tstandard\n'
        'pexpert/qcom_usb_debug/soc/QcomUsbQualify.c\t\t\tstandard\n'
        'pexpert/qcom_usb_debug/soc/QcomUsbDmaDomain.c\t\tstandard\n'
        'pexpert/qcom_usb_debug/Pclm10BootReset.c\t\t\tstandard',
        'pexpert/conf/files QUD units',
    )

    qud_warn_suppress = (
        '-Wno-reserved-identifier -Wno-pre-c11-compat -Wno-switch-default '
        '-Wno-declaration-after-statement -Wno-missing-prototypes '
        '-Wno-unsafe-buffer-usage -Wno-implicit-int-conversion -Wno-shorten-64-to-32'
    )
    replace_exact(
        xnu / 'pexpert/conf/Makefile.template',
        'pe_serial.o_CWARNFLAGS_ADD += -Wno-sign-conversion',
        'pe_serial.o_CWARNFLAGS_ADD += -Wno-sign-conversion\n'
        f'QcomXnuUsbDebug.o_CWARNFLAGS_ADD += {qud_warn_suppress}\n'
        f'QcomUsbScreenshot.o_CWARNFLAGS_ADD += {qud_warn_suppress}\n'
        f'Dwc3.o_CWARNFLAGS_ADD += {qud_warn_suppress}\n'
        f'QcomUsbQualify.o_CWARNFLAGS_ADD += {qud_warn_suppress}\n'
        f'QcomUsbDmaDomain.o_CWARNFLAGS_ADD += {qud_warn_suppress}\n'
        f'Pclm10BootReset.o_CWARNFLAGS_ADD += {qud_warn_suppress}',
        'pexpert/conf/Makefile.template QUD warn suppress',
    )

    # Console & kprintf hooks (preserving fbcon PE_kputc when serial_init succeeded)
    pe_kprintf = xnu / 'pexpert/arm/pe_kprintf.c'
    replace_exact(
        pe_kprintf,
        '#include <libkern/section_keywords.h>',
        '#include <libkern/section_keywords.h>\n'
        '#if XNU_QCOM_USB_DEBUG\n'
        '#include <QcomXnuUsbDebug.h>\n'
        '#endif',
        'pe_kprintf.c include',
    )
    replace_exact(
        pe_kprintf,
        'static SIMPLE_LOCK_DECLARE(kprintf_lock, 0);',
        '#if XNU_QCOM_USB_DEBUG\n'
        'static bool qcom_usb_debug_kprintf_requested;\n'
        'static void\n'
        'qcom_usb_tee(char c)\n'
        '{\n'
        '\tif (PE_kputc == console_write_unbuffered) {\n'
        '\t\tPE_kputc(c);\n'
        '\t\treturn;\n'
        '\t}\n'
        '\tqcom_usb_debug_putc(c);\n'
        '\tif (PE_kputc && PE_kputc != qcom_usb_debug_putc) {\n'
        '\t\tPE_kputc(c);\n'
        '\t}\n'
        '}\n'
        '#define QCOM_USB_DEBUG_KPRINTF_PUTC qcom_usb_tee\n'
        '#else\n'
        '#define QCOM_USB_DEBUG_KPRINTF_PUTC PE_kputc\n'
        '#endif\n\n'
        'static SIMPLE_LOCK_DECLARE(kprintf_lock, 0);',
        'pe_kprintf.c tee helper',
    )
    replace_exact(
        pe_kprintf,
        '\tif (PE_parse_boot_argn("-enable_kprintf_spam", &enable_kprintf_spam, sizeof(enable_kprintf_spam))) {\n'
        '\t\tdisable_kprintf_output = false;\n'
        '\t}\n'
        '#endif\n'
        '}',
        '\tif (PE_parse_boot_argn("-enable_kprintf_spam", &enable_kprintf_spam, sizeof(enable_kprintf_spam))) {\n'
        '\t\tdisable_kprintf_output = false;\n'
        '\t}\n'
        '#endif\n'
        '#if XNU_QCOM_USB_DEBUG\n'
        '\tqcom_usb_debug_kprintf_requested = !disable_kprintf_output;\n'
        '\tdisable_kprintf_output = true;\n'
        '#endif\n'
        '}',
        'pe_kprintf.c defer spam',
    )
    replace_exact(
        pe_kprintf,
        '\t} else {\n'
        '\t\t/**\n'
        '\t\t * If serial failed to initialize then fall back to using the console,\n'
        '\t\t * and assume the console is using the video console (because clearly\n'
        '\t\t * serial doesn\'t work).\n'
        '\t\t */\n'
        '\t\tPE_kputc = console_write_unbuffered;\n'
        '\t}\n'
        '}',
        '\t} else {\n'
        '\t\t/**\n'
        '\t\t * If serial failed to initialize then fall back to using the console,\n'
        '\t\t * and assume the console is using the video console (because clearly\n'
        '\t\t * serial doesn\'t work).\n'
        '\t\t */\n'
        '\t\tPE_kputc = console_write_unbuffered;\n'
        '\t}\n'
        '#if XNU_QCOM_USB_DEBUG\n'
        '\tif (qcom_usb_debug_init() && !PE_kputc) {\n'
        '\t\tPE_kputc = qcom_usb_debug_putc;\n'
        '\t}\n'
        '\tdisable_kprintf_output = !qcom_usb_debug_kprintf_requested;\n'
        '#endif\n'
        '}',
        'pe_kprintf.c qcom_usb_debug_init',
    )
    replace_exact(
        pe_kprintf,
        '\t_doprnt_log(format, &listp, PE_kputc, 16);',
        '\t_doprnt_log(format, &listp, QCOM_USB_DEBUG_KPRINTF_PUTC, 16);',
        'pe_kprintf.c _kprintf sink',
    )
    replace_exact(
        pe_kprintf,
        '\t\t_doprnt_log(fmt, &listp, PE_kputc, 16);',
        '\t\t_doprnt_log(fmt, &listp, QCOM_USB_DEBUG_KPRINTF_PUTC, 16);',
        'pe_kprintf.c kprintf sink',
    )

    # debug.c panic prepare + drain (including panic_spin_shmcon and kdp_machine_reboot_type!)
    debug_c = xnu / 'osfmk/kern/debug.c'
    replace_exact(
        debug_c,
        'void\npanic_trap_to_debugger(const char *panic_format_str, va_list *panic_args, unsigned int reason, void *ctx,',
        '#if XNU_QCOM_USB_DEBUG\n#include <QcomXnuUsbDebug.h>\n#endif\n\n'
        'void\npanic_trap_to_debugger(const char *panic_format_str, va_list *panic_args, unsigned int reason, void *ctx,',
        'debug.c QUD include',
    )
    replace_exact(
        debug_c,
        '#pragma clang diagnostic pop\n\n#if defined(SWEET)',
        '#pragma clang diagnostic pop\n\n'
        '#if XNU_QCOM_USB_DEBUG\n\tqcom_usb_debug_panic_prepare();\n#endif\n\n'
        '#if defined(SWEET)',
        'debug.c panic_prepare',
    )
    replace_exact(
        debug_c,
        '#if defined(SWEET)\n\t\textern void pclm10_panic_autoreset(void);\n\t\tpclm10_panic_autoreset();\n#endif\n\t\t/* If panic debugging is configured',
        '#if XNU_QCOM_USB_DEBUG\n\t\tqcom_usb_debug_panic_drain();\n#endif\n'
        '#if defined(SWEET)\n\t\textern void pclm10_panic_autoreset(void);\n\t\tpclm10_panic_autoreset();\n#endif\n\t\t/* If panic debugging is configured',
        'debug.c panic_drain before shmcon',
    )
    replace_exact(
        debug_c,
        '\tif (!panicDebugging) {\n#if defined(SWEET)\n\t\textern void pclm10_panic_autoreset(void);\n\t\tpclm10_panic_autoreset();\n#endif\n\t\tkdp_machine_reboot_type(',
        '\tif (!panicDebugging) {\n#if XNU_QCOM_USB_DEBUG\n\t\tqcom_usb_debug_panic_drain();\n#endif\n#if defined(SWEET)\n\t\textern void pclm10_panic_autoreset(void);\n\t\tpclm10_panic_autoreset();\n#endif\n\t\tkdp_machine_reboot_type(',
        'debug.c panic_drain before reboot_type',
    )
    replace_exact(
        debug_c,
        'void\npanic_spin_forever(void)\n{\n#if defined(SWEET)\n\textern void pclm10_panic_autoreset(void);\n\tpclm10_panic_autoreset();\n#endif\n\tfor (;;) {',
        'void\npanic_spin_forever(void)\n{\n#if XNU_QCOM_USB_DEBUG\n\tqcom_usb_debug_panic_drain();\n#endif\n#if defined(SWEET)\n\textern void pclm10_panic_autoreset(void);\n\tpclm10_panic_autoreset();\n#endif\n\tfor (;;) {',
        'debug.c panic_drain before spin_forever',
    )

    # printf.c consdebug_putc / consdebug_putc_unbuffered
    printf_c = xnu / 'osfmk/kern/printf.c'
    replace_exact(
        printf_c,
        'void\nconsdebug_putc(char c)\n{\n',
        '#if XNU_QCOM_USB_DEBUG\n#include <QcomXnuUsbDebug.h>\n#endif\n\nvoid\nconsdebug_putc(char c)\n{\n'
        '#if XNU_QCOM_USB_DEBUG\n'
        '\tconst bool usb_via_pe_console = !console_is_serial() && !disable_serial_output &&\n'
        '\t    PE_kputc == console_write_unbuffered;\n'
        '\tif (disableConsoleOutput && !usb_via_pe_console) {\n'
        '\t\tqcom_usb_debug_putc(c);\n'
        '\t}\n'
        '\tif (!usb_via_pe_console)\n'
        '#endif\n',
        'printf.c consdebug_putc pre',
    )
    replace_exact(
        printf_c,
        '\tif (!console_is_serial() && !disable_serial_output) {\n\t\tPE_kputc(c);\n\t}\n}\n\nvoid\nconsdebug_putc_unbuffered(char c)\n{\n'
        '\tif (!disableConsoleOutput) {\n\t\tconsole_write_unbuffered(c);\n\t}\n\n\tdebug_putc(c);\n\n'
        '\tif (!console_is_serial() && !disable_serial_output) {\n\t\tPE_kputc(c);\n\t}',
        '\tif (!console_is_serial() && !disable_serial_output) {\n'
        '#if XNU_QCOM_USB_DEBUG\n\t\tif (PE_kputc && PE_kputc != qcom_usb_debug_putc)\n#endif\n'
        '\t\tPE_kputc(c);\n\t}\n}\n\nvoid\nconsdebug_putc_unbuffered(char c)\n{\n'
        '#if XNU_QCOM_USB_DEBUG\n'
        '\tconst bool usb_via_pe_console = !console_is_serial() && !disable_serial_output &&\n'
        '\t    PE_kputc == console_write_unbuffered;\n'
        '\tif (disableConsoleOutput && !usb_via_pe_console) {\n'
        '\t\tqcom_usb_debug_putc(c);\n'
        '\t}\n'
        '\tif (!usb_via_pe_console)\n'
        '#endif\n'
        '\tif (!disableConsoleOutput) {\n\t\tconsole_write_unbuffered(c);\n\t}\n\n\tdebug_putc(c);\n\n'
        '\tif (!console_is_serial() && !disable_serial_output) {\n'
        '#if XNU_QCOM_USB_DEBUG\n\t\tif (PE_kputc && PE_kputc != qcom_usb_debug_putc)\n#endif\n'
        '\t\tPE_kputc(c);\n\t}',
        'printf.c consdebug_putc_unbuffered',
    )

    # serial_console.c
    serial_console = xnu / 'osfmk/console/serial_console.c'
    replace_exact(
        serial_console,
        '#include <libkern/section_keywords.h>',
        '#include <libkern/section_keywords.h>\n#if XNU_QCOM_USB_DEBUG\n#include <QcomXnuUsbDebug.h>\n#endif',
        'serial_console.c include',
    )
    replace_exact(
        serial_console,
        'console_write(char *str, int size)\n{\n\tassert(str != NULL);',
        'console_write(char *str, int size)\n{\n\tassert(str != NULL);\n\n'
        '#if XNU_QCOM_USB_DEBUG\n'
        '\tfor (int offset = 0; offset < size;) {\n'
        '\t\tint bytes = size - offset;\n'
        '\t\tif (bytes > 16384) {\n'
        '\t\t\tbytes = 16384;\n'
        '\t\t}\n'
        '\t\t(void)qcom_usb_debug_write(str + offset, (size_t)bytes);\n'
        '\t\toffset += bytes;\n'
        '\t}\n'
        '#endif',
        'serial_console.c console_write',
    )
    replace_exact(
        serial_console,
        'console_write_unbuffered(char c)\n{\n\t_cnputc(c, true);',
        'console_write_unbuffered(char c)\n{\n#if XNU_QCOM_USB_DEBUG\n\tqcom_usb_debug_putc(c);\n#endif\n\t_cnputc(c, true);',
        'serial_console.c console_write_unbuffered',
    )

    # serial_general.c
    replace_exact(
        xnu / 'osfmk/console/serial_general.c',
        '\t\tif (serialmode & SERIALMODE_SYNCDRAIN) {',
        '#if !XNU_QCOM_USB_DEBUG\n\t\tif (serialmode & SERIALMODE_SYNCDRAIN)\n#endif\n\t\t{',
        'serial_general.c syncdrain bypass',
    )

    # bsd/kern/bsd_init.c worker + autoreset + control REBOOT
    bsd_init = xnu / 'bsd/kern/bsd_init.c'
    replace_exact(
        bsd_init,
        '#include <kern/sched_hygiene.h>',
        '#include <kern/sched_hygiene.h>\n'
        '#if XNU_QCOM_USB_DEBUG\n'
        '#include <QcomXnuUsbDebug.h>\n'
        '#include <kern/cpu_number.h>\n'
        '#include <kern/processor.h>\n'
        '#include <kern/debug.h>\n'
        '#endif',
        'bsd_init.c QUD includes',
    )
    worker_code = '''#if XNU_QCOM_USB_DEBUG
extern char pclm10_usb_persist_line[384];
extern int boot_reset_bootloader(char *, unsigned);
extern void pclm10_delay_ms(uint32_t ms);
extern processor_t thread_bind(processor_t processor);
extern void thread_terminate_self(void);

static unsigned int qcom_usb_worker_state;
static unsigned int qcom_usb_reboot_state;

static void
qcom_usb_worker_wait(void)
{
\tuint64_t deadline;
\tclock_interval_to_deadline(5, NSEC_PER_MSEC, &deadline);
\tif (assert_wait_deadline((event_t)&qcom_usb_worker_state,
\t    THREAD_UNINT, deadline) == THREAD_WAITING) {
\t\tthread_block(THREAD_CONTINUE_NULL);
\t}
}

static void
qcom_usb_reboot_worker(void *argument __unused, wait_result_t result __unused)
{
\tprocessor_t processor = cpu_to_processor(master_cpu);
\tif (processor != PROCESSOR_NULL) {
\t\tthread_bind(processor);
\t\tthread_block(THREAD_CONTINUE_NULL);
\t}
\tif (!qcom_usb_debug_control_arm_reboot()) {
\t\t__atomic_store_n(&qcom_usb_reboot_state, 4u, __ATOMIC_RELEASE);
\t\tqcom_usb_debug_control_reboot_failed(-4);
\t\tthread_terminate_self();
\t\treturn;
\t}
\tqcom_usb_debug_control_reset_now();
\t__atomic_store_n(&qcom_usb_reboot_state, 4u, __ATOMIC_RELEASE);
\tqcom_usb_debug_control_reboot_failed(-5);
\tthread_terminate_self();
}

static void
qcom_usb_start_reboot_once(void)
{
\tthread_t thread = THREAD_NULL;
\tunsigned int state, running = 1u;
\tkern_return_t kr;
\tif (!qcom_usb_debug_control_take_reboot()) {
\t\treturn;
\t}
\t(void)qcom_usb_debug_wait_drained(2000);
\tfor (;;) {
\t\tstate = __atomic_load_n(&qcom_usb_reboot_state, __ATOMIC_ACQUIRE);
\t\tif (state != 0u && state != 3u && state != 4u) {
\t\t\tqcom_usb_debug_control_reboot_failed(-6);
\t\t\treturn;
\t\t}
\t\tif (__atomic_compare_exchange_n(&qcom_usb_reboot_state, &state, 1u,
\t\t    0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
\t\t\tbreak;
\t\t}
\t}
\tkr = kernel_thread_start(qcom_usb_reboot_worker, NULL, &thread);
\tif (kr == KERN_SUCCESS) {
\t\t(void)__atomic_compare_exchange_n(&qcom_usb_reboot_state, &running,
\t\t    2u, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
\t\tthread_deallocate(thread);
\t} else {
\t\t__atomic_store_n(&qcom_usb_reboot_state, 3u, __ATOMIC_RELEASE);
\t\tqcom_usb_debug_control_reboot_failed(kr);
\t}
}

static void
qcom_usb_worker(void *argument __unused, wait_result_t result __unused)
{
\tstruct qcom_usb_debug_status status = {0};
\tprocessor_t processor = cpu_to_processor(master_cpu);
\tint control_enabled = 0;
\tint control_active = 0;
\tunsigned int autoreset_sec = 0;
\tunsigned int idle_ticks = 0;

\tthread_set_thread_name(current_thread(), "qcom-usb-debug");
\tif (processor == PROCESSOR_NULL) {
\t\t__atomic_store_n(&qcom_usb_worker_state, 3u, __ATOMIC_RELEASE);
\t\tgoto stopped;
\t}
\tthread_bind(processor);
\tthread_block(THREAD_CONTINUE_NULL);
\tif (cpu_number() != master_cpu) {
\t\t__atomic_store_n(&qcom_usb_worker_state, 3u, __ATOMIC_RELEASE);
\t\tgoto stopped;
\t}

\tqcom_usb_debug_get_status(&status);
\tprintf("PCLM10 USB: %s (phase=%u err=%u qcom=%u dom=%u stg=%u lt=%u a=%x b=%x tr=%u)\\n",
\t    pclm10_usb_persist_line[0] ? pclm10_usb_persist_line : "(empty)",
\t    status.phase, status.error, status.qcom_result,
\t    status.domain_result, status.domain_stage,
\t    status.last_trace, status.trace_a, status.trace_b, status.traces);
\t(void)PE_parse_boot_argn("pclm10-autoreset", &autoreset_sec, sizeof(autoreset_sec));
\tif (status.phase != QCOM_USB_DEBUG_ACTIVE) {
\t\tif (autoreset_sec > 0u) {
\t\t\tunsigned int wait_sec = autoreset_sec < 8u ? autoreset_sec : 8u;
\t\t\tchar rbuf[160];
\t\t\tprintf("PCLM10 AUTORESET: QUD phase=%u err=%u; rebooting in %us\\n",
\t\t\t    status.phase, status.error, wait_sec);
\t\t\tpclm10_delay_ms(wait_sec * 1000u);
\t\t\t(void)boot_reset_bootloader(rbuf, sizeof(rbuf));
\t\t}
\t\tgoto stopped;
\t}
\t__atomic_store_n(&qcom_usb_worker_state, 2u, __ATOMIC_RELEASE);
\tkprintf("QCOM USB WORKER active cpu=%d period_ms=5 buffer=%llu\\n",
\t    cpu_number(), status.buffer_capacity);
\tif (PE_parse_boot_argn("qcom-usb-ctl", &control_enabled, sizeof(control_enabled)) &&
\t    control_enabled == 1) {
\t\tif (qcom_usb_debug_control_start()) {
\t\t\tcontrol_active = 1;
\t\t\tqcom_usb_debug_get_status(&status);
\t\t\tkprintf("QCOM USB CONTROL active hook=%u\\n", status.control_hook_owned);
\t\t}
\t}
\tfor (;;) {
\t\tif (panic_active()) {
\t\t\tbreak;
\t\t}
\t\tqcom_usb_debug_get_status(&status);
\t\tif (status.phase != QCOM_USB_DEBUG_ACTIVE) {
\t\t\tbreak;
\t\t}
\t\tqcom_usb_debug_poll();
\t\tif (control_active) {
\t\t\tqcom_usb_start_reboot_once();
\t\t}
\t\tif (autoreset_sec > 0u && status.control_replies == 0 &&
\t\t    ++idle_ticks >= autoreset_sec * 200u) {
\t\t\tchar rbuf[160];
\t\t\tprintf("PCLM10 AUTORESET: no USB host after %us; rebooting to fastboot\\n", autoreset_sec);
\t\t\t(void)boot_reset_bootloader(rbuf, sizeof(rbuf));
\t\t\tbreak;
\t\t}
\t\tqcom_usb_worker_wait();
\t}

stopped:
\tif (__atomic_load_n(&qcom_usb_worker_state, __ATOMIC_ACQUIRE) != 3u) {
\t\t__atomic_store_n(&qcom_usb_worker_state, 4u, __ATOMIC_RELEASE);
\t}
\tthread_bind(PROCESSOR_NULL);
\tthread_terminate_self();
}

static void
qcom_usb_start_worker_once(void)
{
\tthread_t thread = THREAD_NULL;
\tunsigned int expected = 0;
\tint enabled = 0;
\tkern_return_t result;

\tif (!PE_parse_boot_argn("qcom-usb-debug", &enabled, sizeof(enabled)) || enabled != 1) {
\t\treturn;
\t}
\tif (!__atomic_compare_exchange_n(&qcom_usb_worker_state, &expected, 1u,
\t    0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
\t\treturn;
\t}
\tresult = kernel_thread_start(qcom_usb_worker, NULL, &thread);
\tif (result == KERN_SUCCESS) {
\t\tthread_deallocate(thread);
\t} else {
\t\t__atomic_store_n(&qcom_usb_worker_state, 3u, __ATOMIC_RELEASE);
\t}
}
#endif /* XNU_QCOM_USB_DEBUG */

kern_return_t
bsd_autoconf(void)
{
\tkprintf("bsd_autoconf: calling kminit\\n");
\tkminit();
#if XNU_QCOM_USB_DEBUG
\tqcom_usb_start_worker_once();
#endif'''
    replace_exact(
        bsd_init,
        'kern_return_t\nbsd_autoconf(void)\n{\n\tkprintf("bsd_autoconf: calling kminit\\n");\n\tkminit();',
        worker_code,
        'bsd_init.c bsd_autoconf QUD worker',
    )


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--xnu', type=Path, required=True, help='Path to patched ios-sweet xnu tree')
    p.add_argument('--qud-dir', type=Path, default=None, help='Prepared QcomXnuUsbDebug tree')
    p.add_argument('--boot-reset-c', type=Path, default=None, help='Path to Pclm10BootReset.c')
    args = p.parse_args()
    apply_overlay(args.xnu)
    if args.qud_dir is not None:
        apply_qud_overlay(args.xnu, args.qud_dir, args.boot_reset_c)
        print(f'Applied QcomXnuUsbDebug overlay cleanly onto {args.xnu}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
