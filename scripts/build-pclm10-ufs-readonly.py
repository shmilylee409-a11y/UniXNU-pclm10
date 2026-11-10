#!/usr/bin/env python3
"""Build a strictly read-only SM8150 UFS probe archive for OPPO Reno Ace (PCLM10).

Reads upstream UniXNU drivers/ShengUfs without modifying it, copies it into
build/sheng-ufs/source/, adapts the SM8550 constants to PCLM10/SM8150 hardware
facts (oppo-pclm10-19081.dtb + gcc-sm8150.c), strips all write paths and
IOBlockStorageDevice/VerifiedRoot root-mount code under PCLM10_UFS_PROBE_ONLY=1,
and compiles build/sheng-ufs/libShengUfs.a with -Werror against XNU's exact
IOPlatformExpert compile record.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))

SM8150_GCC_UFS_H = """// SPDX-License-Identifier: BSD-2-Clause-Patent
// SM8150 (OPPO Reno Ace PCLM10) GCC UFS registers from Linux gcc-sm8150.c.
// Strictly read-only: NativeUfs probe only inspects clock/power gates.
#ifndef NATIVE_UFS_GCC_UFS_H
#define NATIVE_UFS_GCC_UFS_H

#define GCC_BASE                      0x00100000ULL
#define UFS_HC_BASE                   0x01d84000ULL
#define UFS_HC_SIZE                   0x3000ULL

#define GCC_UFS_PHY_GDSCR             0x77004
#define GCC_UFS_MEM_PHY_GDSCR         0x77004

#define GCC_UFS_PHY_AXI_ENABLE        0x77010
#define GCC_UFS_PHY_AHB_ENABLE        0x77014
#define GCC_UFS_PHY_TX_SYMBOL0_ENABLE 0x77018
#define GCC_UFS_PHY_RX_SYMBOL0_ENABLE 0x7701c
#define GCC_UFS_PHY_UNIPRO_CORE_ENABLE 0x77058
#define GCC_UFS_PHY_ICE_CORE_ENABLE   0x7705c
#define GCC_UFS_PHY_PHY_AUX_ENABLE    0x77090
#define GCC_UFS_PHY_RX_SYMBOL1_ENABLE 0x770ac
#define GCC_AGGRE_UFS_PHY_AXI_ENABLE  0x770c0

#define GCC_UFS_BRANCH_ENABLE_BIT     0x1U

#endif
"""


def transform_source(src_dir: Path, dst_dir: Path) -> None:
    if dst_dir.exists():
        shutil.rmtree(dst_dir)
    shutil.copytree(src_dir, dst_dir, ignore=shutil.ignore_patterns(".git", "tests"))

    (dst_dir / "GccUfs.h").write_text(SM8150_GCC_UFS_H, encoding="utf-8")

    cpp_path = dst_dir / "NativeUfs.cpp"
    text = cpp_path.read_text(encoding="utf-8")

    def replace_exact(old: str, new: str, label: str) -> None:
        nonlocal text
        count = text.count(old)
        if count != 1:
            raise SystemExit(f"Expected exactly 1 match for {label}, found {count}")
        text = text.replace(old, new, 1)

    # 1. Remove IOStorageFamily header in probe-only mode; add stream helper + console output.
    replace_exact(
        "#include <IOKit/storage/IOBlockStorageDevice.h>",
        "#if !defined(PCLM10_UFS_PROBE_ONLY)\n#include <IOKit/storage/IOBlockStorageDevice.h>\n#endif\n#include <sys/systm.h>",
        "IOBlockStorageDevice include",
    )

    replace_exact(
        "static void rootDiagnosticFailure (const char *reason);",
        """static void rootDiagnosticFailure (const char *reason);
static uint16_t sm8150UfsStream (void)
{
  uint32_t stream = 0x300U;
  PE_parse_boot_argn ("native-ufs-stream", &stream, sizeof (stream));
  return static_cast<uint16_t> (stream & 0x7fffU);
}""",
        "sm8150UfsStream helper",
    )

    # 2. Match SM8150 / generic ufshc DT node.
    replace_exact(
        'match = text && contains (text, data->getLength (), "qcom,sm8550-ufshc");',
        'match = text && (contains (text, data->getLength (), "qcom,sm8150-ufshc") || contains (text, data->getLength (), "qcom,ufshc"));',
        "OSData compatible match",
    )
    replace_exact(
        'match = text && contains (text, string->getLength (), "qcom,sm8550-ufshc");',
        'match = text && (contains (text, string->getLength (), "qcom,sm8150-ufshc") || contains (text, string->getLength (), "qcom,ufshc"));',
        "OSString compatible match",
    )

    # 3. SM8150 inspect(): do not map SM8550-only GCC_BASE + 0x9e000; log before panic.
    old_inspect = """void NativeUfs::inspect (void)
{
  OSSharedPtr<IOMemoryMap> gccMap, memPhyMap, hcMap;
  uintptr_t gcc = mapPhys (GCC_BASE + 0x77000, 0x1000, gccMap);
  uintptr_t memPhy = mapPhys (GCC_BASE + 0x9e000, 0x1000, memPhyMap);
  if (!gcc || !memPhy) {
    panic ("NATIVE_UFS inspect: GCC map failed gcc=%p memphy=%p", (void *) gcc, (void *) memPhy);
  }
  uint32_t gdsc = read32 (gcc, GCC_UFS_PHY_GDSCR - 0x77000);
  uint32_t axi = read32 (gcc, GCC_UFS_PHY_AXI_ENABLE - 0x77000);
  uint32_t ahb = read32 (gcc, GCC_UFS_PHY_AHB_ENABLE - 0x77000);
  uint32_t unipro = read32 (gcc, GCC_UFS_PHY_UNIPRO_CORE_ENABLE - 0x77000);
  uint32_t txs = read32 (gcc, GCC_UFS_PHY_TX_SYMBOL0_ENABLE - 0x77000);
  uint32_t rxs = read32 (gcc, GCC_UFS_PHY_RX_SYMBOL0_ENABLE - 0x77000);
  uint32_t aux = read32 (gcc, GCC_UFS_PHY_PHY_AUX_ENABLE - 0x77000);
  uint32_t memGdsc = read32 (memPhy, 0);"""
    new_inspect = """void NativeUfs::inspect (void)
{
  PE_initialize_console (nullptr, kPETextScreen);
  OSSharedPtr<IOMemoryMap> gccMap, hcMap;
  uintptr_t gcc = mapPhys (GCC_BASE + 0x77000, 0x1000, gccMap);
  if (!gcc) {
    panic ("NATIVE_UFS inspect: GCC map failed gcc=%p", (void *) gcc);
  }
  uint32_t gdsc = read32 (gcc, GCC_UFS_PHY_GDSCR - 0x77000);
  uint32_t axi = read32 (gcc, GCC_UFS_PHY_AXI_ENABLE - 0x77000);
  uint32_t ahb = read32 (gcc, GCC_UFS_PHY_AHB_ENABLE - 0x77000);
  uint32_t unipro = read32 (gcc, GCC_UFS_PHY_UNIPRO_CORE_ENABLE - 0x77000);
  uint32_t txs = read32 (gcc, GCC_UFS_PHY_TX_SYMBOL0_ENABLE - 0x77000);
  uint32_t rxs = read32 (gcc, GCC_UFS_PHY_RX_SYMBOL0_ENABLE - 0x77000);
  uint32_t aux = read32 (gcc, GCC_UFS_PHY_PHY_AUX_ENABLE - 0x77000);
  uint32_t memGdsc = 0;"""
    replace_exact(old_inspect, new_inspect, "inspect memPhy removal")

    replace_exact(
        '  panic ("NATIVE_UFS inspect\\n"',
        '  printf ("NATIVE_UFS inspect: gdsc=%08x axi=%08x ahb=%08x hc=%u cap=%08x ver=%08x hcs=%08x hce=%08x ba=%08x\\n",\n'
        '          gdsc, axi, ahb, readHc, cap, ver, hcs, hce, utrlba);\n'
        '  panic ("NATIVE_UFS inspect\\n"',
        "inspect printf",
    )

    # 4. adoptAndRead(): allow IOBufferMemoryDescriptor fallback if firmware UTRLBA is 0/unmappable,
    #    and use SM8150 stream ID instead of hardcoded 0x60.
    old_fwlist = """  uint64_t fwList = (uint64_t (read32 (hc, 0x54)) << 32) | read32 (hc, 0x50);
  if (!fwList || (fwList & 0xfff)) {
    panic ("NATIVE_UFS adopt: no usable firmware list base=%llx", fwList);
  }
  IOMemoryDescriptor *region = IOMemoryDescriptor::withPhysicalAddress (fwList, 16384, kIODirectionInOut);
  OSSharedPtr<IOMemoryMap> regionMap;
  if (region) {
    regionMap = OSSharedPtr<IOMemoryMap> (region->map (kIOMapAnywhere), OSNoRetain);
    region->release ();
  }
  if (!regionMap) {
    panic ("NATIVE_UFS adopt: firmware list map failed base=%llx", fwList);
  }
  uint8_t *virt = reinterpret_cast<uint8_t *> (regionMap->getVirtualAddress ());
  uint64_t phys = fwList;"""
    new_fwlist = """  uint64_t fwList = (uint64_t (read32 (hc, 0x54)) << 32) | read32 (hc, 0x50);
  unsigned forceAlloc = 0;
  PE_parse_boot_argn ("native-ufs-probe-alloc", &forceAlloc, sizeof (forceAlloc));
  OSSharedPtr<IOMemoryMap> regionMap;
  IOBufferMemoryDescriptor *allocDesc = nullptr;
  uint8_t *virt = nullptr;
  uint64_t phys = 0;
  if (!forceAlloc && fwList && !(fwList & 0xfff)) {
    IOMemoryDescriptor *region = IOMemoryDescriptor::withPhysicalAddress (fwList, 16384, kIODirectionInOut);
    if (region) {
      regionMap = OSSharedPtr<IOMemoryMap> (region->map (kIOMapAnywhere), OSNoRetain);
      region->release ();
      if (regionMap) {
        virt = reinterpret_cast<uint8_t *> (regionMap->getVirtualAddress ());
        phys = fwList;
      }
    }
  }
  if (!virt) {
    allocDesc = IOBufferMemoryDescriptor::inTaskWithOptions (
      kernel_task,
      static_cast<IOOptionBits> (kIODirectionInOut) | static_cast<IOOptionBits> (kIOMemoryPhysicallyContiguous) |
      static_cast<IOOptionBits> (kIOMemoryMapperNone),
      16384, 4096);
    if (allocDesc) {
      IOByteCount segLen = 0;
      virt = static_cast<uint8_t *> (allocDesc->getBytesNoCopy ());
      phys = allocDesc->getPhysicalSegment (0, &segLen, kIOMemoryMapperNone);
      if (!virt || !phys || (phys & 0xfff) || segLen < 16384) {
        allocDesc->release ();
        allocDesc = nullptr;
        virt = nullptr;
      }
    }
  }
  if (!virt) {
    panic ("NATIVE_UFS adopt: DMA buffer unavailable fw=%llx", fwList);
  }"""
    replace_exact(old_fwlist, new_fwlist, "adoptAndRead DMA fallback")

    replace_exact(
        "  smmuBypassStream (0x60, smmuMap, smmuReport, phys, 16384);",
        "  smmuBypassStream (sm8150UfsStream (), smmuMap, smmuReport, phys, 16384);",
        "adoptAndRead stream ID",
    )

    # 5. inspectSmmu(): check SM8150 stream (0x300) and report active SMRs.
    old_smmu_scan = """  unsigned match = 0xffff, valid = 0, typeCount[4] = { 0, 0, 0, 0 };
  uint32_t matchSmr = 0, matchS2cr = 0;
  for (unsigned index = 0; index < count; index++) {
    uint32_t smr = read32 (smmu, 0x800 + 4 * index);
    uint32_t s2cr = read32 (smmu, 0xc00 + 4 * index);
    if (smr & 0x80000000U) {
      valid++;
      typeCount[(s2cr >> 16) & 3]++;
      uint32_t mask = (smr >> 16) & 0x7fff;
      if (match == 0xffff && (((smr ^ 0x60U) & 0x7fffU) & ~mask) == 0) {
        match = index;
        matchSmr = smr;
        matchS2cr = s2cr;
      }
    }
  }
  panic ("NATIVE_UFS smmu\\n"
         "scr0=%08x idr0=%08x idr1=%08x gfsr=%08x\\n"
         "nsmr=%u valid=%u types t0=%u t1=%u t2=%u\\n"
         "sid0x60 match=%x smr=%08x s2cr=%08x",
         scr0, idr0, idr1, gfsr, count, valid, typeCount[0], typeCount[1], typeCount[2],
         match, matchSmr, matchS2cr);"""
    new_smmu_scan = """  unsigned match = 0xffff, valid = 0, typeCount[4] = { 0, 0, 0, 0 };
  uint32_t matchSmr = 0, matchS2cr = 0;
  uint32_t firstIdx[4] = { 0xffff, 0xffff, 0xffff, 0xffff };
  uint32_t firstSmr[4] = { 0, 0, 0, 0 };
  uint32_t firstS2cr[4] = { 0, 0, 0, 0 };
  unsigned seenValid = 0;
  for (unsigned index = 0; index < count; index++) {
    uint32_t smr = read32 (smmu, 0x800 + 4 * index);
    uint32_t s2cr = read32 (smmu, 0xc00 + 4 * index);
    if (smr & 0x80000000U) {
      if (seenValid < 4) {
        firstIdx[seenValid] = index;
        firstSmr[seenValid] = smr;
        firstS2cr[seenValid] = s2cr;
        seenValid++;
      }
      valid++;
      typeCount[(s2cr >> 16) & 3]++;
      uint32_t mask = (smr >> 16) & 0x7fff;
      if (match == 0xffff && (((smr ^ uint32_t (sm8150UfsStream ())) & 0x7fffU) & ~mask) == 0) {
        match = index;
        matchSmr = smr;
        matchS2cr = s2cr;
      }
    }
  }
  panic ("NATIVE_UFS smmu\\n"
         "scr0=%08x idr0=%08x idr1=%08x gfsr=%08x\\n"
         "nsmr=%u valid=%u types t0=%u t1=%u t2=%u\\n"
         "sid0x%x match=%x smr=%08x s2cr=%08x\\n"
         "v0(%x:%08x/%08x) v1(%x:%08x/%08x)\\n"
         "v2(%x:%08x/%08x) v3(%x:%08x/%08x)",
         scr0, idr0, idr1, gfsr, count, valid, typeCount[0], typeCount[1], typeCount[2],
         sm8150UfsStream (), match, matchSmr, matchS2cr,
         firstIdx[0], firstSmr[0], firstS2cr[0], firstIdx[1], firstSmr[1], firstS2cr[1],
         firstIdx[2], firstSmr[2], firstS2cr[2], firstIdx[3], firstSmr[3], firstS2cr[3]);"""
    replace_exact(old_smmu_scan, new_smmu_scan, "inspectSmmu stream ID")

    # 6. Guard root storage / IOBlockStorageDevice / write-window sections out in PCLM10_UFS_PROBE_ONLY mode.
    replace_exact(
        "// ======================================================================================\n// Root media: the live controller as a read-only block device.",
        "#if !defined(PCLM10_UFS_PROBE_ONLY)\n// ======================================================================================\n// Root media: the live controller as a read-only block device.",
        "probe-only block guard start",
    )
    replace_exact(
        "bool NativeUfs::start (IOService *provider)",
        """#else
static void rootDiagnosticFailure (const char *reason) { native_ufs_root_failure = reason; }
#endif

bool NativeUfs::start (IOService *provider)""",
        "probe-only block guard end",
    )
    replace_exact(
        "  if (probe == 10) {",
        "#if !defined(PCLM10_UFS_PROBE_ONLY)\n  if (probe == 10) {",
        "start() probe>=10 guard start",
    )
    replace_exact(
        "  if (rootMode && !probe) {\n    return startRoot (provider);\n  }\n  return false;\n}\n\n// Boot the root image's windows as block devices. Soft-fails (logs, publishes nothing).\nbool NativeUfs::startRoot (IOService *provider)",
        """  if (rootMode && !probe) {
    return startRoot (provider);
  }
#endif
  return false;
}

#if !defined(PCLM10_UFS_PROBE_ONLY)
// Boot the root image's windows as block devices. Soft-fails (logs, publishes nothing).
bool NativeUfs::startRoot (IOService *provider)""",
        "start() guard end + startRoot guard start",
    )
    replace_exact(
        "void NativeUfs::stop (IOService *provider)",
        """#else
bool NativeUfs::startRoot (IOService *) { return false; }
#endif

void NativeUfs::stop (IOService *provider)""",
        "startRoot probe-only stub",
    )

    # Safety check: probe-only code outside #if !defined(PCLM10_UFS_PROBE_ONLY) must not contain WRITE(10).
    pre_guard = text.split("#if !defined(PCLM10_UFS_PROBE_ONLY)", 1)[0]
    probe_body = text.split("bool NativeUfs::start (IOService *provider)", 1)[1]
    if "0x2a" in pre_guard.lower() or "0x2a" in probe_body.lower():
        raise SystemExit("Safety refusal: WRITE(10) opcode 0x2a found in read-only probe path!")

    cpp_path.write_text(text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare-only", type=Path, help="Transform into this dir and exit without compiling (host test)")
    parser.add_argument("--source", type=Path, default=None)
    args = parser.parse_args()

    src_dir = args.source
    if src_dir is None:
        fallback = ROOT / "build/sheng-ufs/upstream"
        candidates = [
            ROOT / "drivers/ShengUfs",
            ROOT / ".cache/pclm10-ShengUfs",
            fallback,
        ]
        # In format-2 builds, ROOT is out/<board>/<os>/<config>/tree while pre-staged
        # caches can live under the UniXNU repo root or UNIXNU_DEVICE.
        if len(ROOT.parents) >= 5:
            candidates.append(ROOT.parents[4] / ".cache/pclm10-ShengUfs")
        if os.environ.get("UNIXNU_DEVICE"):
            candidates.append(Path(os.environ["UNIXNU_DEVICE"]) / ".cache/pclm10-ShengUfs")
        for candidate in candidates:
            if (candidate / "NativeUfs.cpp").is_file():
                src_dir = candidate
                break
        if src_dir is None:
            if fallback.exists():
                shutil.rmtree(fallback)
            fallback.parent.mkdir(parents=True, exist_ok=True)
            subprocess.run(
                ["git", "clone", "https://github.com/UniXNU/ShengUfs.git", str(fallback)],
                check=True,
            )
            subprocess.run(
                ["git", "-C", str(fallback), "checkout", "99d8bcd3e43dd24fc3c8437c67b5d4cc71ab94d0"],
                check=True,
            )
            src_dir = fallback

    if args.prepare_only:
        transform_source(src_dir.resolve(), args.prepare_only.resolve())
        print(f"Prepared SM8150 read-only UFS probe source at {args.prepare_only}")
        return 0

    from kernel_target import TRIPLE, check_object  # imported only during kernel build inside UniXNU/scripts

    recipe = ROOT / "build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/iokit/DEVELOPMENT/IOPlatformExpert.cpo.json"
    record = json.loads(recipe.read_text().strip().rstrip(","))

    out = ROOT / "build/sheng-ufs"
    source_dir = out / "source"
    transform_source(src_dir.resolve(), source_dir)

    cpp = source_dir / "NativeUfs.cpp"
    obj = out / "NativeUfs.o"
    archive = out / "libShengUfs.a"

    command = []
    arguments = record["arguments"]
    i = 0
    while i < len(arguments):
        val = arguments[i]
        if val == record["file"]:
            command.append(str(cpp))
        elif val == "-o":
            command.extend(["-o", str(obj)])
            i += 1
        elif val in ("-MF", "-MJ"):
            i += 1
        elif val == "-Xclang" and i + 1 < len(arguments) and arguments[i + 1].startswith("-W"):
            i += 1
        elif not val.startswith("-W") and val not in ("-MD", "-MP"):
            command.append(val)
        i += 1

    command.extend([
        "-DPCLM10_UFS_PROBE_ONLY=1",
        "-DNATIVE_VERIFIED_BLOCK=0",
        "-Wall", "-Wextra", "-Werror",
        "-Wno-unused-parameter",
        "-Wno-unused-function",
        "-Wno-unused-variable",
        "-Wno-c99-designator",
        "-Wno-reorder-init-list",
        "-Wno-deprecated-enum-enum-conversion",
        "-Wno-unnecessary-virtual-specifier",
        "-mgeneral-regs-only",
        "-I", str(ROOT / ".cache/xnu-work/osfmk"),
    ])
    (out / "compile-command.json").write_text(json.dumps(command, indent=2) + "\n")
    res = subprocess.run(command, cwd=record["directory"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (out / "compile.log").write_text(res.stdout)
    if res.returncode:
        raise SystemExit(res.stdout[-4000:])

    check_object(obj.read_bytes(), "NativeUfs")
    archive.unlink(missing_ok=True)
    subprocess.run(["xcrun", "ar", "rcs", archive, obj], check=True)
    (out / "build-report.json").write_text(json.dumps({
        "mode": "pclm10-sm8150-readonly-probe",
        "source": str(src_dir),
        "archive": str(archive),
        "archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        "triple": TRIPLE,
    }, indent=2) + "\n")
    print(f"ShengUfs (PCLM10 SM8150 read-only probe): -Werror build and {TRIPLE} object audit passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
