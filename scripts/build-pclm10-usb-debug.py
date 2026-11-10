#!/usr/bin/env python3
"""Build QcomXnuUsbDebug (SM8150) + Pclm10BootReset archive for OPPO Reno Ace (PCLM10).

Adapts upstream FFT-infra/QcomXnuUsbDebug (pinned commit 18bcc385) with:
  1. Verified SM8150 hardware profile soc/Sm8150.h (oppo-pclm10-19081.dtb + gcc-sm8150.c).
  2. Unattended panic control loop in qcom_usb_debug_panic_drain() when qcom-usb-ctl=1
     is passed, so early probes (e.g. native-ufs-probe=2/7/9 before bsd_autoconf)
     remain reachable over USB CDC for status/screenshot/reboot-to-fastboot.
  3. Lockless Pclm10BootReset.o providing _boot_reset_bootloader() via IMEM + PM8150
     SPMI PON + SCM + PSCI + PS_HOLD + APSS WDT fallback.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
QUD_REPO = "https://github.com/FFT-infra/QcomXnuUsbDebug.git"
QUD_COMMIT = "18bcc385bc0c97ff38343a83040f19711392867b"

SM8150_H = """/* SPDX-License-Identifier: MIT
 * SM8150 / OPPO Reno Ace (PCLM10) profile for QcomXnuUsbDebug.
 *
 * Verified from:
 *   - dts/oppo-pclm10-19081.dtb (/soc/ssusb@a600000, /soc/apps-smmu@0x15000000,
 *     /soc/qcom,gcc@100000, /soc/qcom,msm-imem@146bf000)
 *   - mainline Linux drivers/clk/qcom/gcc-sm8150.c
 */
#ifndef QUD_SM8150_H
#define QUD_SM8150_H
#define QUD_SOC_NAME "sm8150-pclm10"

#define QUD_USB_VID 0x1209u
#define QUD_USB_PID 0x316du
#define QUD_USB_SERIAL "QcomXnuUsbDebug"
#define QUD_USB_PRODUCT "Qualcomm XNU Debug"

/* GCC clock-controller@100000, reg <0x100000 0x1f0000> */
#define QUD_GCC_BASE 0x00100000ULL
#define QUD_GCC_BYTES 0x1f0000u

/* ssusb@a600000, reg <0xa600000 0x100000>, iommus = <0x26 0x140 0x0> */
#define QUD_DWC3_BASE 0x0a600000ULL
#define QUD_DWC3_BYTES 0x100000u

/* apps-smmu@15000000, reg <0x15000000 0x100000> */
#define QUD_SMMU_BASE 0x15000000ULL
#define QUD_SMMU_BYTES 0x100000ULL
#define QUD_STREAM_ID 0x140u

/* Private 2 MiB-aligned 32-bit IOVA for DWC3 DMA arena. */
#define QUD_DMA_IOVA 0x90000000ULL

/* Linux drivers/clk/qcom/gcc-sm8150.c:
 *   gcc_usb30_prim_master_clk     = 0xf010
 *   gcc_usb30_prim_sleep_clk      = 0xf014
 *   gcc_usb30_prim_mock_utmi_clk  = 0xf018
 *   gcc_usb3_prim_phy_aux_clk     = 0xf050
 *   gcc_usb3_prim_phy_com_aux_clk = 0xf054
 *   gcc_usb3_prim_phy_pipe_clk    = 0xf058 (BRANCH_HALT_SKIP, omitted)
 */
#define QUD_GCC_CBCR_OFFSETS {0xf010u, 0xf014u, 0xf018u, 0xf050u, 0xf054u}
#define QUD_GCC_CBCR_REQUIRED 3u

/* GCC_USB30_PRIM_BCR = 0xf000; usb30_prim_gdsc .gdscr = 0xf004, .flags = POLL_CFG_GDSCR */
#define QUD_GCC_BCR_OFFSET 0xf000u
#define QUD_GCC_GDSCR_OFFSET 0xf004u
#define QUD_GCC_CFG_GDSCR_OFFSET 0xf008u

/* QSCRATCH_HS_PHY_CTRL = ssusb@a600000 + 0xf8810 */
#define QUD_QSCRATCH_HS_PHY_CTRL_OFFSET 0xf8810u
#define QUD_QSCRATCH_VBUS_MASK ((1u << 20) | (1u << 28))

/* Control opt-in allowed to install PE_halt_restart when qcom-usb-ctl=1 */
#define QUD_PSCI_SMC_RESET 1
#endif
"""

PANIC_CTL_ANCHOR = """out:
    buffer_status_owned();
    release(flags);
    if (usb_diag_enabled) diag_reset(10);
}"""

PANIC_CTL_REPLACEMENT = """out:
    buffer_status_owned();
    release(flags);
    if (usb_diag_enabled) diag_reset(10);
    {
        int ctl_opt = 0;
        if (PE_parse_boot_argn("qcom-usb-ctl", &ctl_opt, sizeof(ctl_opt)) && ctl_opt == 1 &&
            qcom_usb_debug_control_start()) {
            for (;;) {
                qcom_usb_debug_poll();
                if (qcom_usb_debug_control_take_reboot()) {
                    (void)qcom_usb_debug_wait_drained(1000);
                    if (qcom_usb_debug_control_arm_reboot()) {
                        qcom_usb_debug_control_reset_now();
                    }
                }
                delay_us(0, 1000);
            }
        }
    }
}"""


def find_qud_source() -> Path:
    fallback = ROOT / "build/qcom-usb-debug/upstream"
    candidates = [
        ROOT / ".cache/pclm10-QcomXnuUsbDebug",
        fallback,
    ]
    if len(ROOT.parents) >= 5:
        candidates.append(ROOT.parents[4] / ".cache/pclm10-QcomXnuUsbDebug")
    if os.environ.get("UNIXNU_DEVICE"):
        candidates.append(Path(os.environ["UNIXNU_DEVICE"]) / ".cache/pclm10-QcomXnuUsbDebug")
    for c in candidates:
        if (c / "kernel/QcomXnuUsbDebug.c").is_file() and (c / "scripts/build.py").is_file():
            return c
    if fallback.exists():
        shutil.rmtree(fallback)
    fallback.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["git", "clone", QUD_REPO, str(fallback)], check=True)
    subprocess.run(["git", "-C", str(fallback), "checkout", QUD_COMMIT], check=True)
    return fallback


def find_boot_reset_source() -> Path:
    candidates = [
        Path(__file__).resolve().parents[1] / "package/Pclm10BootReset.c",
        ROOT / ".cache/pclm10-Pclm10BootReset.c",
    ]
    if len(ROOT.parents) >= 5:
        candidates.append(ROOT.parents[4] / ".cache/pclm10-Pclm10BootReset.c")
    if os.environ.get("UNIXNU_DEVICE"):
        candidates.append(Path(os.environ["UNIXNU_DEVICE"]) / "package/Pclm10BootReset.c")
    for c in candidates:
        if c.is_file():
            return c
    raise SystemExit("Unable to locate package/Pclm10BootReset.c")


def prepare_source(src_dir: Path, dst_dir: Path) -> None:
    if dst_dir.exists():
        shutil.rmtree(dst_dir)
    shutil.copytree(src_dir, dst_dir, ignore=shutil.ignore_patterns(".git", "tmp", ".cache"))
    (dst_dir / "soc/Sm8150.h").write_text(SM8150_H, encoding="utf-8")

    qud_c = dst_dir / "kernel/QcomXnuUsbDebug.c"
    text = qud_c.read_text(encoding="utf-8")
    if text.count(PANIC_CTL_ANCHOR) != 1:
        raise SystemExit("Expected exactly 1 match for PANIC_CTL_ANCHOR in QcomXnuUsbDebug.c")
    qud_c.write_text(text.replace(PANIC_CTL_ANCHOR, PANIC_CTL_REPLACEMENT, 1), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare-only", type=Path, help="Stage adapted QUD source into dir and exit")
    args = parser.parse_args()

    src_dir = find_qud_source()
    if args.prepare_only:
        prepare_source(src_dir, args.prepare_only.resolve())
        print(f"Prepared SM8150 QcomXnuUsbDebug source at {args.prepare_only}")
        return 0

    from kernel_target import TRIPLE, check_object  # imported only inside UniXNU build tree

    out = ROOT / "build/qcom-usb-debug"
    out.mkdir(parents=True, exist_ok=True)
    source_dir = out / "source"
    prepare_source(src_dir, source_dir)

    recipe = ROOT / "build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/pexpert/DEVELOPMENT/pe_kprintf.o.json"
    if not recipe.is_file():
        matches = list((ROOT / "build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837").rglob("pe_kprintf.o.json"))
        if not matches:
            raise SystemExit("pe_kprintf.o.json not found in build/xnu/ios-obj")
        recipe = matches[0]

    ar = shutil.which("llvm-ar") or shutil.which("ar")
    nm = shutil.which("llvm-nm") or shutil.which("nm")
    if not ar or not nm:
        raise SystemExit(f"Missing archiver/nm tools: ar={ar} nm={nm}")

    cache_out = out / ".cache/build"
    subprocess.run(
        [
            sys.executable,
            str(source_dir / "scripts/build.py"),
            "--compile-record", str(recipe),
            "--soc", "soc/Sm8150.h",
            "--ar", ar,
            "--nm", nm,
            "--replace",
            "--output", str(cache_out),
        ],
        check=True,
    )

    spec = importlib.util.spec_from_file_location("qud_build", source_dir / "scripts/build.py")
    qud_build = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(qud_build)
    record = json.loads(recipe.read_text(encoding="utf-8").strip().rstrip(","))
    flags, cwd = qud_build.compile_flags(record)

    boot_reset_src = find_boot_reset_source()
    boot_reset_copy = out / "Pclm10BootReset.c"
    shutil.copy2(boot_reset_src, boot_reset_copy)
    boot_reset_obj = out / "Pclm10BootReset.o"
    cmd = flags + ["-Wno-switch-default", "-Wno-unused-function", "-c", str(boot_reset_copy), "-o", str(boot_reset_obj)]
    res = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (out / "boot-reset-compile.log").write_text(res.stdout, encoding="utf-8")
    if res.returncode:
        raise SystemExit(res.stdout[-3500:])
    check_object(boot_reset_obj.read_bytes(), "Pclm10BootReset")

    archive = out / "libQcomXnuUsbDebug.a"
    shutil.copy2(cache_out / "libQcomXnuUsbDebug.a", archive)
    subprocess.run([ar, "rcs", str(archive), str(boot_reset_obj)], check=True)

    inc_dst = out / "include"
    if inc_dst.exists():
        shutil.rmtree(inc_dst)
    shutil.copytree(source_dir / "include", inc_dst)

    (out / "build-report.json").write_text(
        json.dumps(
            {
                "soc": "sm8150-pclm10",
                "qud_source": str(src_dir),
                "boot_reset_source": str(boot_reset_src),
                "archive": str(archive),
                "archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
                "triple": TRIPLE,
            },
            indent=2,
        ) + "\n",
        encoding="utf-8",
    )
    print(f"QcomXnuUsbDebug (SM8150 + Pclm10BootReset): build and {TRIPLE} object audit passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
