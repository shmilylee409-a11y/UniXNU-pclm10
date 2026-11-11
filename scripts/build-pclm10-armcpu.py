#!/usr/bin/env python3
"""Build SM8150-adapted ArmCpu archive for OPPO Reno Ace (PCLM10).

Adapts UniXNU/drivers/ArmCpu/ArmCpu.cpp with:
  1. SM8150 virtual timer INTID 19 support (reads /arm-io/intc/virtual-timer-intid,
     defaulting to 19 per SM8150 Qualcomm HYP hardware instead of hardcoded 27).
  2. Darwin ARM64 inline assembly newline separator fix (LLVM treats ';' as an asm
     comment on Darwin Mach-O, silently dropping trailing instructions such as
     isb/dsb and icc_sgi1r_el1 in multi-instruction one-liners).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import struct
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

TIMER_HELPER = r'''extern "C" {
volatile uint32_t armcpu_gic_stage = 0;
volatile uint32_t armcpu_pclm10_status = 0;
void pclm10_wake_usb_worker (void) __attribute__((weak));
}

static uint32_t gTimerIntid = 19;
static uint32_t kSgiPpiEnable = 0xffffU | (1U << 19);

static bool pclm10TimerIntid(bool present, const void *data, unsigned size, uint32_t *out)
{
  if (!present) { *out = 19; return true; }
  if (!data || size != 4) { return false; }
  const uint8_t *bytes = static_cast<const uint8_t *>(data);
  uint32_t value = uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) |
                   (uint32_t(bytes[2]) << 16) | (uint32_t(bytes[3]) << 24);
  if (value < 16 || value >= 32) { return false; }
  *out = value;
  return true;
}
'''

PREPARE_GIC_REPLACEMENT = r'''bool ArmCpu::prepareGic (void)
{
  armcpu_gic_stage = 1;
  if (!mapGic ()) {
    __atomic_store_n (&armcpu_pclm10_status, 2U, __ATOMIC_RELEASE);
    IOLog ("PCLM10 ARMCPU FAIL: mapGic failed stage=%u\n", armcpu_gic_stage);
    if (pclm10_wake_usb_worker) { pclm10_wake_usb_worker (); }
    return false;
  }
  armcpu_gic_stage = 2;
  if (!initDistributor ()) {
    __atomic_store_n (&armcpu_pclm10_status, 2U, __ATOMIC_RELEASE);
    IOLog ("PCLM10 ARMCPU FAIL: initDistributor failed stage=%u ctl=0x%x\n",
           armcpu_gic_stage, read32 (dist, 0));
    if (pclm10_wake_usb_worker) { pclm10_wake_usb_worker (); }
    return false;
  }
  armcpu_gic_stage = 3;
  if (!wakeRedistributor (true)) {
    __atomic_store_n (&armcpu_pclm10_status, 2U, __ATOMIC_RELEASE);
    IOLog ("PCLM10 ARMCPU FAIL: wakeRedistributor failed stage=%u aff=0x%llx redist=0x%llx typer=0x%x\n",
           armcpu_gic_stage,
           static_cast<unsigned long long> (affinity),
           static_cast<unsigned long long> (redist),
           armcpu_gic_typer_probe[1]);
    if (pclm10_wake_usb_worker) { pclm10_wake_usb_worker (); }
    return false;
  }
  armcpu_gic_stage = 4;
  gGicShared = 1;
  return true;
}'''

START_VERIFY_SUFFIX = r'''  __atomic_store_n (&gBootRegistered, 1U, __ATOMIC_RELEASE);
  drainPending ();
  if (!skipGic) {
    uint64_t freq = 0, start_t = 0, now_t = 0;
    __asm__ volatile ("mrs %0, cntfrq_el0" : "=r" (freq));
    __asm__ volatile ("isb\n\tmrs %0, cntvct_el0" : "=r" (start_t));
    if (freq == 0) { freq = 19200000ULL; }
    uint64_t limit = start_t + (freq / 5ULL);
    while (__atomic_load_n (&armcpu_timer_interrupts, __ATOMIC_ACQUIRE) == 0) {
      __asm__ volatile ("isb\n\tmrs %0, cntvct_el0" : "=r" (now_t));
      if (now_t >= limit) { break; }
    }
    uint64_t tirq = __atomic_load_n (&armcpu_timer_interrupts, __ATOMIC_ACQUIRE);
    if (tirq == 0) {
      uint64_t daif = 0, ctl = 0, cval = 0, cnt = 0, hppir = 0, rpr = 0;
      __asm__ volatile ("mrs %0, daif" : "=r" (daif));
      __asm__ volatile ("mrs %0, cntv_ctl_el0" : "=r" (ctl));
      __asm__ volatile ("mrs %0, cntv_cval_el0" : "=r" (cval));
      __asm__ volatile ("mrs %0, cntvct_el0" : "=r" (cnt));
      __asm__ volatile ("mrs %0, icc_hppir1_el1" : "=r" (hppir));
      __asm__ volatile ("mrs %0, icc_rpr_el1" : "=r" (rpr));
      uint32_t isp = redist ? read32 (redist + 0x10000, 0x200) : 0;
      uint32_t iact = redist ? read32 (redist + 0x10000, 0x280) : 0;
      IOLog ("PCLM10 ARMCPU TIMER TIMEOUT: intid=%u daif=0x%llx ctl=0x%llx cval=0x%llx cnt=0x%llx hppir=0x%llx rpr=0x%llx isp=0x%x iact=0x%x boot=%x/%x/%x/%x/%x/%x/%x/%x\n",
             gTimerIntid,
             static_cast<unsigned long long> (daif),
             static_cast<unsigned long long> (ctl),
             static_cast<unsigned long long> (cval),
             static_cast<unsigned long long> (cnt),
             static_cast<unsigned long long> (hppir),
             static_cast<unsigned long long> (rpr),
             isp, iact,
             armcpu_gic_boot[0], armcpu_gic_boot[1], armcpu_gic_boot[2], armcpu_gic_boot[3],
             armcpu_gic_boot[4], armcpu_gic_boot[5], armcpu_gic_boot[6], armcpu_gic_boot[7]);
      __atomic_store_n (&armcpu_pclm10_status, 2U, __ATOMIC_RELEASE);
    } else {
      IOLog ("PCLM10 ARMCPU OK: intid=%u tirq=%llu boot=%x/%x/%x/%x/%x/%x\n",
             gTimerIntid,
             static_cast<unsigned long long> (tirq),
             armcpu_gic_boot[0], armcpu_gic_boot[1], armcpu_gic_boot[2],
             armcpu_gic_boot[5], armcpu_gic_boot[6], armcpu_gic_boot[7]);
      __atomic_store_n (&armcpu_pclm10_status, 1U, __ATOMIC_RELEASE);
    }
    if (pclm10_wake_usb_worker) { pclm10_wake_usb_worker (); }
  }
  return true;'''

TIMER_PROPERTY = r'''  OSObject *timerObject = gic->getProperty ("virtual-timer-intid");
  OSData *timerData = OSDynamicCast (OSData, timerObject);
  uint32_t timerIntid = 19;
  valid = valid && pclm10TimerIntid(timerObject != nullptr,
      timerData ? timerData->getBytesNoCopy() : nullptr,
      timerData ? timerData->getLength() : 0, &timerIntid);
  if (valid) {
    gTimerIntid = timerIntid;
    kSgiPpiEnable = 0xffffU | (1U << timerIntid);
    IOLog("ArmCpu: virtual timer INTID=%u\n", timerIntid);
  }
'''

ASM_TEMPLATES = (
    (r'isb; mrs %0, cntvct_el0', 1),
    (r'msr icc_sre_el1, %0; isb', 1),
    (r'msr icc_igrpen1_el1, %0; dsb sy; isb', 2),
    (r'msr icc_eoir1_el1, %0; isb', 2),
    (r'dsb ishst; msr icc_sgi1r_el1, %0; isb', 2),
    (r'mrs %0, daif; msr daifset, #2; isb', 1),
    (r'dsb ishst; isb; mrs %0, cntvct_el0; isb', 1),
    (r'msr icc_sgi1r_el1, %0; isb; mrs %1, cntvct_el0', 1),
    (r'dsb ishst; msr icc_sgi0r_el1, %0; isb', 1),
    (r'msr icc_eoir0_el1, %0; isb', 1),
    (r'isb; mrs %0, cntvct_el0; isb; mrs %2, icc_pmr_el1; isb; mrs %1, cntvct_el0', 1),
    (r'msr daif, %0; isb', 1),
    (r'msr cntv_ctl_el0, %0; isb', 1),
    (r'dsb sy; isb', 1),
)


def patch_timer_source(text: str) -> str:
    for old, new in (
        ('static const uint32_t kSgiPpiEnable = 0xffffU | (1U << 27);\n', TIMER_HELPER),
        ('  armcpu_gic_stride = static_cast<uint32_t> (gRedistStride);\n',
         TIMER_PROPERTY + '  armcpu_gic_stride = static_cast<uint32_t> (gRedistStride);\n'),
        ('    if ((id & 0xffffff) == 27) {\n', '    if ((id & 0xffffff) == gTimerIntid) {\n'),
        ('bool ArmCpu::prepareGic (void)\n{\n  if (!mapGic () || !initDistributor () || !wakeRedistributor (true)) { return false; }\n  gGicShared = 1;\n  return true;\n}',
         PREPARE_GIC_REPLACEMENT),
        ('  if (!valid || (regions[1] < 0x10000) || (regions[3] < 0x20000)) {\n    return false;\n  }',
         '  if (!valid) {\n    IOLog ("PCLM10 ARMCPU FAIL: mapGic dt invalid\\n");\n    return false;\n  }\n  if ((regions[1] < 0x10000) || (regions[3] < 0x20000)) {\n    IOLog ("PCLM10 ARMCPU FAIL: mapGic regions r1=0x%llx r3=0x%llx\\n", static_cast<unsigned long long> (regions[1]), static_cast<unsigned long long> (regions[3]));\n    return false;\n  }'),
        ('  if (((read32 (dist, 0xffe8) >> 4) & 15) != 3) {\n    return false;\n  }',
         '  if (((read32 (dist, 0xffe8) >> 4) & 15) != 3) {\n    IOLog ("PCLM10 ARMCPU FAIL: mapGic pidr2=0x%x\\n", read32 (dist, 0xffe8));\n    return false;\n  }'),
        ('  __atomic_store_n (&gBootRegistered, 1U, __ATOMIC_RELEASE);\n  drainPending ();\n  return true;',
         START_VERIFY_SUFFIX),
        ('    if ((!skipGic && !prepareGic ()) || !IOCPU::start (provider)) { return false; }',
         '    if (!skipGic && !prepareGic ()) { return false; }\n    if (!IOCPU::start (provider)) {\n      __atomic_store_n (&armcpu_pclm10_status, 2U, __ATOMIC_RELEASE);\n      IOLog ("PCLM10 ARMCPU FAIL: IOCPU::start failed\\n");\n      if (pclm10_wake_usb_worker) { pclm10_wake_usb_worker (); }\n      return false;\n    }'),
    ):
        if text.count(old) != 1:
            raise SystemExit(f"ArmCpu timer source anchor mismatch: {old[:80]!r}")
        text = text.replace(old, new, 1)
    return text


def patch_assembly(text: str) -> str:
    for template, expected_count in ASM_TEMPLATES:
        old = '"' + template + '"'
        actual = text.count(old)
        if actual != expected_count:
            raise SystemExit(f"ArmCpu asm anchor mismatch ({actual} != {expected_count}): {template!r}")
        fixed = template.replace('; ', r'\n\t')
        if template.startswith('msr icc_sgi1r_el1, %0;'):
            fixed = r'msr icc_sgi1r_el1, %1\n\tisb\n\tmrs %0, cntvct_el0'
        text = text.replace(old, '"' + fixed + '"')
    if re.search(r'__asm__\s+volatile\s*\(\s*"[^"\n]*;', text):
        raise SystemExit('ArmCpu retains a Darwin inline-asm ";" comment separator')
    return text


def patch_source(text: str) -> str:
    return patch_assembly(patch_timer_source(text))


def check_ipi_object(obj_bytes: bytes) -> dict:
    """Verify real dsb ishst / msr icc_sgi1r_el1 / isb sequence inside ArmCpu.o."""
    if struct.unpack_from('<4I', obj_bytes, 0) != (0xfeedfacf, 0x100000c, 0, 1):
        raise SystemExit('Expected ARM64 MH_OBJECT for ArmCpu.o')
    ncmds = struct.unpack_from('<I', obj_bytes, 16)[0]
    at = 32
    sections = []
    symtab_off = nsyms = stroff = strsize = 0
    for _ in range(ncmds):
        cmd, cmdsize = struct.unpack_from('<II', obj_bytes, at)
        if cmd == 0x19:  # LC_SEGMENT_64
            nsects = struct.unpack_from('<I', obj_bytes, at + 64)[0]
            sat = at + 72
            for _ in range(nsects):
                addr, size, offset = struct.unpack_from('<QQI', obj_bytes, sat + 32)
                sections.append((addr, size, offset))
                sat += 80
        elif cmd == 0x2:  # LC_SYMTAB
            symtab_off, nsyms, stroff, strsize = struct.unpack_from('<4I', obj_bytes, at + 8)
        at += cmdsize
    strtab = obj_bytes[stroff:stroff + strsize]
    names = {}
    for i in range(nsyms):
        n_strx, n_type, _, _, n_value = struct.unpack_from('<IBBHQ', obj_bytes, symtab_off + i * 16)
        if (n_type & 0x0e) == 0x0e:
            end = strtab.find(b'\x00', n_strx)
            names[strtab[n_strx:end].decode('utf-8', errors='replace')] = n_value
    checked = {}
    for sym in ('__ZN6ArmCpu9signalCPUEP5IOCPU', '_armcpu_ipi_selftest'):
        start = names.get(sym)
        if start is None:
            raise SystemExit(f'Missing ArmCpu IPI symbol: {sym}')
        sec = next(s for s in sections if s[0] <= start < s[0] + s[1])
        end = min([v for v in names.values() if start < v < sec[0] + sec[1]] + [sec[0] + sec[1]])
        code = obj_bytes[sec[2] + start - sec[0]:sec[2] + end - sec[0]]
        words = struct.unpack(f'<{len(code) // 4}I', code)
        hits = [i for i, w in enumerate(words) if (w & 0xffffffe0) == 0xd518cba0]
        if not any(i > 0 and i + 1 < len(words) and words[i - 1] == 0xd5033a9f and words[i + 1] == 0xd5033fdf for i in hits):
            raise SystemExit(f'ArmCpu IPI function lacks dsb/icc_sgi1r_el1/isb: {sym}')
        checked[sym] = [hex(start + 4 * i) for i in hits]
    return checked


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT / 'drivers/ArmCpu/ArmCpu.cpp')
    parser.add_argument('--prepare-only', type=Path, help='Write patched ArmCpu.cpp to path and exit')
    args = parser.parse_args()

    src_text = args.source.read_text(encoding='utf-8')
    patched_text = patch_source(src_text)
    if args.prepare_only:
        args.prepare_only.parent.mkdir(parents=True, exist_ok=True)
        args.prepare_only.write_text(patched_text, encoding='utf-8')
        print(f'Prepared SM8150 ArmCpu source at {args.prepare_only}')
        return 0

    from kernel_target import TRIPLE, check_object  # imported only inside UniXNU build tree

    recipe = ROOT / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/iokit/DEVELOPMENT/IOPlatformExpert.cpo.json'
    record = json.loads(recipe.read_text().strip().rstrip(','))

    out = ROOT / 'build/armcpu'
    out.mkdir(parents=True, exist_ok=True)
    cpp = out / 'ArmCpu.cpp'
    cpp.write_text(patched_text, encoding='utf-8')
    obj = out / 'ArmCpu.o'
    archive = out / 'libArmCpu.a'

    command = []
    arguments = record['arguments']
    i = 0
    while i < len(arguments):
        val = arguments[i]
        if val == record['file']:
            command.append(str(cpp))
        elif val == '-o':
            command.extend(['-o', str(obj)])
            i += 1
        elif val in ('-MF', '-MJ'):
            i += 1
        elif val == '-Xclang' and i + 1 < len(arguments) and arguments[i + 1].startswith('-W'):
            i += 1
        elif not val.startswith('-W') and val not in ('-MD', '-MP'):
            command.append(val)
        i += 1

    command.extend([
        '-Wall', '-Wextra', '-Werror',
        '-Wno-unused-parameter',
        '-Wno-unused-function',
        '-Wno-c99-designator',
        '-Wno-reorder-init-list',
        '-Wno-deprecated-enum-enum-conversion',
        '-Wno-unnecessary-virtual-specifier',
        '-mgeneral-regs-only',
        '-I', str(ROOT / '.cache/xnu-work/osfmk'),
    ])
    (out / 'compile-command.json').write_text(json.dumps(command, indent=2) + '\n', encoding='utf-8')
    res = subprocess.run(command, cwd=record['directory'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (out / 'compile.log').write_text(res.stdout, encoding='utf-8')
    if res.returncode:
        raise SystemExit(res.stdout[-4000:])

    obj_bytes = obj.read_bytes()
    check_object(obj_bytes, 'ArmCpu')
    ipi_hits = check_ipi_object(obj_bytes)
    archive.unlink(missing_ok=True)
    subprocess.run(['xcrun', 'ar', 'rcs', str(archive), str(obj)], check=True)
    (out / 'build-report.json').write_text(json.dumps({
        'mode': 'pclm10-sm8150-armcpu',
        'timer_intid_default': 19,
        'ipi_hits': ipi_hits,
        'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
        'triple': TRIPLE,
    }, indent=2) + '\n', encoding='utf-8')
    print(f'ArmCpu (PCLM10 SM8150 INTID 19 + Darwin asm): -Werror build and {TRIPLE} audit passed')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
