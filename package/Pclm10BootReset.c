// SPDX-License-Identifier: MIT
// Lockless reset-into-fastboot implementation for OPPO Reno Ace (PCLM10 / SM8150).
//
// Safe in both normal kernel thread context (qcom_usb_reboot_worker) and
// interrupt-masked panic context (panic_spin_forever -> qcom_usb_debug_panic_drain).
// Avoids IOLockAlloc/IOSleep completely; uses CNTVCT_EL0 delays and
// ml_io_map_unmappable(..., 7 /* VM_WIMG_IO */).
//
// Verified hardware addresses from dts/oppo-pclm10-19081.dtb:
//   /soc/qcom,msm-imem@146bf000/restart_reason@65c -> 0x146bf65c (0x77665500 = bootloader)
//   /soc/qcom,spmi@c440000 (core=0xc440000, chnls=0xc600000, obsrvr=0xe600000, cnfg=0xc40a000)
//   /soc/restart@c264000 -> PS_HOLD
//   /soc/qcom,wdt@17c10000 -> APSS KPSS watchdog bite fallback

#include <stdint.h>
#include <stddef.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>

extern vm_offset_t ml_io_map_unmappable(vm_offset_t phys_addr, vm_size_t size, uint32_t flags);

enum {
	BR_IMEM_BASE         = 0x146bf000u,
	BR_IMEM_SIZE         = 0x1000u,
	BR_IMEM_REASON_OFF   = 0x65cu,
	BR_REASON_BOOTLOADER = 0x77665500u,

	BR_PSHOLD_BASE       = 0x0c264000u,
	BR_PSHOLD_SIZE       = 0x1000u,

	BR_WDT_BASE          = 0x17c10000u,
	BR_WDT_SIZE          = 0x1000u,
	BR_WDT_RST           = 0x04u,
	BR_WDT_EN            = 0x08u,
	BR_WDT_BARK          = 0x10u,
	BR_WDT_BITE          = 0x14u,

	SPMI_CORE_BASE       = 0x0c440000u,
	SPMI_CORE_SIZE       = 0x2000u,
	SPMI_CHNLS_BASE      = 0x0c600000u,
	SPMI_CHNL_STRIDE     = 0x10000u,
	SPMI_OBS_BASE        = 0x0e600000u,
	SPMI_OBS_SIZE        = 0x10000u,
	SPMI_OBS_STRIDE      = 0x80u,
	SPMI_CNFG_BASE       = 0x0c40a000u,
	SPMI_CNFG_SIZE       = 0x1000u,

	SPMI_APID_MAP        = 0x900u,
	SPMI_OWNER           = 0x700u,
	SPMI_MAX_APIDS       = 512u,
	SPMI_CMD             = 0x00u,
	SPMI_STATUS          = 0x08u,
	SPMI_WDATA0          = 0x10u,
	SPMI_RDATA0          = 0x18u,
	SPMI_STATUS_DONE     = 0x01u,
	SPMI_STATUS_ERR      = 0x0eu,

	PON_SUBTYPE_ADDR     = 0x805u,
	PON_SPARE_ADDR       = 0x88fu,
	PON_GEN2_SUBTYPE     = 0x04u,
	PON_REASON_BOOTLOADER = 0x02u,

	PSCI_SYSTEM_RESET    = 0x84000009u,
};

struct br_writer {
	char *buf;
	unsigned cap;
	unsigned len;
};

static void br_putc(struct br_writer *w, char c)
{
	if (!w->cap) return;
	if (w->len + 1u < w->cap) {
		w->buf[w->len++] = c;
		w->buf[w->len] = '\0';
	}
}

static void br_puts(struct br_writer *w, const char *s)
{
	while (*s) br_putc(w, *s++);
}

static void br_puthex(struct br_writer *w, uint64_t val, unsigned digits)
{
	static const char hex[] = "0123456789abcdef";
	for (unsigned i = digits; i > 0u; --i) {
		br_putc(w, hex[(val >> ((i - 1u) * 4u)) & 0xfu]);
	}
}

static void br_putdec(struct br_writer *w, unsigned val)
{
	char tmp[16];
	unsigned n = 0;
	if (val == 0) {
		br_putc(w, '0');
		return;
	}
	while (val && n < sizeof(tmp)) {
		tmp[n++] = (char)('0' + (val % 10u));
		val /= 10u;
	}
	while (n--) br_putc(w, tmp[n]);
}

static void udelay_cntvct(uint32_t usec)
{
	uint64_t freq = 0, start = 0, ticks;
	__asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
	if (!freq || freq > 1000000000ULL) freq = 19200000ULL;
	__asm__ volatile("isb\nmrs %0, cntvct_el0" : "=r"(start));
	ticks = (freq * (uint64_t)usec) / 1000000ULL;
	if (!ticks) ticks = 1;
	for (;;) {
		uint64_t now = 0;
		__asm__ volatile("mrs %0, cntvct_el0" : "=r"(now));
		if ((now - start) >= ticks) break;
		__asm__ volatile("yield" ::: "memory");
	}
}

static uint64_t smc_psci(uint64_t fid, uint64_t a1)
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

static uint64_t scm_pwr(uint64_t cmd, uint64_t arg)
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

static inline uint32_t spmi_rd(vm_offset_t base, unsigned off)
{
	uint32_t v = *(volatile uint32_t *)(base + off);
	__asm__ volatile("dmb oshld" ::: "memory");
	return v;
}

static int spmi_find_apid(vm_offset_t core, vm_offset_t cnfg, unsigned count, unsigned ppid)
{
	for (unsigned i = 0; i < count; ++i) {
		uint32_t map = spmi_rd(core, SPMI_APID_MAP + 4u * i);
		if (!map || ((map >> 8) & 0xfffu) != ppid) continue;
		if ((spmi_rd(cnfg, SPMI_OWNER + 4u * i) & 7u) == 0u) return (int)i;
	}
	return -1;
}

static int spmi_obs_read8(vm_offset_t core, vm_offset_t cnfg, vm_offset_t obs,
                          unsigned count, unsigned addr, uint8_t *out)
{
	int apid = spmi_find_apid(core, cnfg, count, (addr >> 8) & 0xfffu);
	if (apid < 0) return 1;
	unsigned off = SPMI_OBS_STRIDE * (unsigned)apid;
	*(volatile uint32_t *)(obs + off + SPMI_CMD) = (1u << 27) | ((addr & 0xffu) << 4);
	__asm__ volatile("dsb osh" ::: "memory");
	for (unsigned i = 0; i < 1000u; ++i) {
		uint32_t st = spmi_rd(obs, off + SPMI_STATUS);
		if (st & SPMI_STATUS_DONE) {
			if (st & SPMI_STATUS_ERR) return 2;
			*out = (uint8_t)spmi_rd(obs, off + SPMI_RDATA0);
			return 0;
		}
		udelay_cntvct(50);
	}
	return 3;
}

static int spmi_chnl_write8(vm_offset_t core, vm_offset_t cnfg, unsigned count,
                            unsigned addr, uint8_t value)
{
	static int cached_apid = -1;
	static vm_offset_t cached_lane = 0;

	int apid = spmi_find_apid(core, cnfg, count, (addr >> 8) & 0xfffu);
	if (apid < 0) return 1;
	if (cached_apid != apid || !cached_lane) {
		if (cached_apid >= 0 && cached_lane) return 4;
		vm_offset_t lane = ml_io_map_unmappable(
		    (vm_offset_t)(SPMI_CHNLS_BASE + SPMI_CHNL_STRIDE * (unsigned)apid),
		    (vm_size_t)SPMI_CHNL_STRIDE, 7u);
		if (!lane) return 5;
		cached_lane = lane;
		cached_apid = apid;
	}
	*(volatile uint32_t *)(cached_lane + SPMI_WDATA0) = value;
	__asm__ volatile("dsb osh" ::: "memory");
	*(volatile uint32_t *)(cached_lane + SPMI_CMD) = ((addr & 0xffu) << 4);
	__asm__ volatile("dsb osh" ::: "memory");
	for (unsigned i = 0; i < 1000u; ++i) {
		uint32_t st = spmi_rd(cached_lane, SPMI_STATUS);
		if (st & SPMI_STATUS_DONE) return (st & SPMI_STATUS_ERR) ? 2 : 0;
		udelay_cntvct(50);
	}
	return 3;
}

static int pon_mark_bootloader(uint8_t *out_subtype, uint8_t *out_before,
                               uint8_t *out_after, int *out_rc)
{
	static vm_offset_t core = 0, cnfg = 0, obs = 0;
	static unsigned count = 0;
	static int ready = 0;

	if (ready < 0) return 1;
	if (ready == 0) {
		core = ml_io_map_unmappable((vm_offset_t)SPMI_CORE_BASE, (vm_size_t)SPMI_CORE_SIZE, 7u);
		cnfg = ml_io_map_unmappable((vm_offset_t)SPMI_CNFG_BASE, (vm_size_t)SPMI_CNFG_SIZE, 7u);
		obs  = ml_io_map_unmappable((vm_offset_t)SPMI_OBS_BASE,  (vm_size_t)SPMI_OBS_SIZE,  7u);
		if (!core || !cnfg || !obs) { ready = -1; return 1; }
		uint32_t ver = spmi_rd(core, 0u);
		count = spmi_rd(core, 4u) & 0x7ffu;
		if (ver < 0x50000000u || ver >= 0x70000000u || !count || count > SPMI_MAX_APIDS) {
			ready = -1;
			return 2;
		}
		ready = 1;
	}
	uint8_t subtype = 0, before = 0, after = 0;
	int rc = spmi_obs_read8(core, cnfg, obs, count, PON_SUBTYPE_ADDR, &subtype);
	*out_subtype = subtype;
	if (rc) { *out_rc = rc; return 3; }
	unsigned shift = (subtype >= PON_GEN2_SUBTYPE) ? 1u : 2u;
	uint8_t mask = (uint8_t)(0xffu << shift);
	uint8_t want = (uint8_t)(PON_REASON_BOOTLOADER << shift);
	(void)spmi_obs_read8(core, cnfg, obs, count, PON_SPARE_ADDR, &before);
	*out_before = before;
	rc = spmi_chnl_write8(core, cnfg, count, PON_SPARE_ADDR,
	                      (uint8_t)((before & (uint8_t)~mask) | want));
	if (!rc) rc = spmi_obs_read8(core, cnfg, obs, count, PON_SPARE_ADDR, &after);
	*out_after = after;
	*out_rc = rc;
	return (!rc && (after & mask) == want) ? 0 : 4;
}

int boot_reset_bootloader(char *text, unsigned cap);
void boot_progress_persist(const char *line, uint32_t color);

char pclm10_usb_persist_line[384];
uint32_t pclm10_usb_persist_color;

void boot_progress_persist(const char *line, uint32_t color)
{
	pclm10_usb_persist_color = color;
	if (!line) {
		pclm10_usb_persist_line[0] = '\0';
		return;
	}
	unsigned i = 0;
	while (line[i] && i + 1u < sizeof(pclm10_usb_persist_line)) {
		pclm10_usb_persist_line[i] = line[i];
		++i;
	}
	pclm10_usb_persist_line[i] = '\0';
}

int boot_reset_bootloader(char *text, unsigned cap)
{
	struct br_writer w = { text, cap, 0 };
	if (text && cap) text[0] = '\0';

	volatile uint8_t *imem = (volatile uint8_t *)ml_io_map_unmappable(
	    (vm_offset_t)BR_IMEM_BASE, (vm_size_t)BR_IMEM_SIZE, 7u);
	if (!imem) {
		br_puts(&w, "reset refused: imem map failed");
		return 1;
	}
	volatile uint32_t *cell = (volatile uint32_t *)(imem + BR_IMEM_REASON_OFF);
	uint32_t imem_before = *cell;
	*cell = BR_REASON_BOOTLOADER;
	__asm__ volatile("dsb sy" ::: "memory");
	uint32_t imem_after = *cell;
	ml_io_unmap((vm_offset_t)imem, (vm_size_t)BR_IMEM_SIZE);

	if (imem_after != BR_REASON_BOOTLOADER) {
		br_puts(&w, "reset refused: imem verify failed ");
		br_puthex(&w, imem_before, 8);
		br_puts(&w, "->");
		br_puthex(&w, imem_after, 8);
		return 11;
	}

	uint8_t pon_sub = 0, pon_before = 0, pon_after = 0;
	int pon_rc = -1;
	int pon_ok = pon_mark_bootloader(&pon_sub, &pon_before, &pon_after, &pon_rc);

	uint64_t arb = scm_pwr(0x1ULL, 0ULL);
	uint64_t psr = scm_pwr(0x2ULL, 0ULL);
	(void)smc_psci(PSCI_SYSTEM_RESET, 0ULL);

	volatile uint32_t *pshold = (volatile uint32_t *)ml_io_map_unmappable(
	    (vm_offset_t)BR_PSHOLD_BASE, (vm_size_t)BR_PSHOLD_SIZE, 7u);
	if (pshold) {
		*pshold = 0u;
		__asm__ volatile("dsb sy" ::: "memory");
		ml_io_unmap((vm_offset_t)pshold, (vm_size_t)BR_PSHOLD_SIZE);
	}
	udelay_cntvct(150000u);

	// Fallback if SCM/PSCI/PS_HOLD all returned: force APSS KPSS watchdog bite (~31ms).
	volatile uint8_t *wdt = (volatile uint8_t *)ml_io_map_unmappable(
	    (vm_offset_t)BR_WDT_BASE, (vm_size_t)BR_WDT_SIZE, 7u);
	if (wdt) {
		*(volatile uint32_t *)(wdt + BR_WDT_EN)   = 0u;
		*(volatile uint32_t *)(wdt + BR_WDT_RST)  = 1u;
		*(volatile uint32_t *)(wdt + BR_WDT_BARK) = 0x400u;
		*(volatile uint32_t *)(wdt + BR_WDT_BITE) = 0x800u;
		*(volatile uint32_t *)(wdt + BR_WDT_EN)   = 1u;
		*(volatile uint32_t *)(wdt + BR_WDT_RST)  = 1u;
		__asm__ volatile("dsb sy" ::: "memory");
		udelay_cntvct(500000u);
	}

	br_puts(&w, "reset returned imem=");
	br_puthex(&w, imem_after, 8);
	br_puts(&w, " pon_ok=");
	br_putdec(&w, (unsigned)(pon_ok == 0 ? 1 : 0));
	br_puts(&w, " pon_rc=");
	br_putdec(&w, (unsigned)(pon_rc < 0 ? 99 : pon_rc));
	br_puts(&w, " sub=");
	br_puthex(&w, pon_sub, 2);
	br_puts(&w, " arb=");
	br_puthex(&w, arb, 8);
	br_puts(&w, " psr=");
	br_puthex(&w, psr, 8);
	return 99;
}
