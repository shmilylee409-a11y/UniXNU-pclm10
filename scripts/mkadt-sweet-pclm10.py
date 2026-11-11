#!/usr/bin/env python3
"""Generate an Apple Device Tree (ADT) for OPPO Reno Ace (PCLM10 / SM8150) ios-sweet XNU."""
from __future__ import annotations

import argparse
import hashlib
import os
import struct
import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS_DIR / "sweet"))
import adt  # noqa: E402

SOC_BASE = 0x01000000
GICD, GICD_SIZE = 0x17a00000, 0x10000
GICR, GICR_SIZE = 0x17a60000, 0x100000
FB, FB_SIZE = 0x9d000000, 0x01000000
FB_W, FB_H, FB_STRIDE = 1080, 2400, 1080 * 4
UFS, UFS_SIZE = 0x01d84000, 0x3000
TIMEBASE = 19200000
VTIMER_INTID = 19
PANIC_LOG_BASE = 0xD8000000
PANIC_LOG_SIZE = 0x100000
PCLM10_ECID = 0x50434C4D31300001  # 'PCLM10' + 1

BOOT_MANIFEST_HASH = hashlib.sha384(b"pclm10 sweet boot manifest").digest()

u32 = lambda *v: struct.pack("<%dI" % len(v), *v)
u64 = lambda *v: struct.pack("<%dQ" % len(v), *v)
cstr = lambda s: s.encode() + b"\0"


def node(name: str, props=(), children=()):
    n = adt.Node()
    n.props = [("name", 0, cstr(name))] + [(k, 0, v) for k, v in props]
    n.children = list(children)
    return n


def d22ap_product(path: Path | None):
    if path is None or not path.is_file():
        return None
    root, _ = adt.parse(path.read_bytes())
    for c in root.children:
        if dict((k, v) for k, f, v in c.props).get("name") == b"product\0":
            return [(k, v) for k, f, v in c.props if not f and k not in ("name", "AAPL,phandle")]
    raise SystemExit(f"{path}: no /product node")


def build(
    *,
    d22ap_adt: Path | None,
    dram_base: int = 0xc0000000,
    dram_size: int = 0x10000000,
    owned_memory_size: int | None = 0x10000000,
    has_ramdisk: bool = False,
    cpus_count: int = 8,
):
    if owned_memory_size is not None:
        if not 0x10000000 <= owned_memory_size <= 0x200000000 or (owned_memory_size & 0x1fffff):
            raise SystemExit("owned-memory-size must be 256 MiB..8 GiB in 2 MiB steps")
    seed = os.urandom(64)
    phandle = iter(range(1, 1000))
    ph = lambda: ("AAPL,phandle", u32(next(phandle)))

    mmap_props = [ph()]
    if has_ramdisk:
        mmap_props.append(("RAMDisk", u64(0, 0)))

    chosen_props = [
        ph(),
        ("dram-base", u64(dram_base)),
        ("dram-size", u64(dram_size)),
        ("random-seed", seed),
        ("debug-enabled", u32(1)),
        ("embedded-panic-log-size", u32(PANIC_LOG_SIZE)),
        ("firmware-version", cstr("pclm10-sweet-uefi")),
        ("system-firmware-version", cstr("pclm10-sweet-uefi")),
        ("boot-manifest-hash", BOOT_MANIFEST_HASH),
        ("crypto-hash-method", cstr("sha2-384")),
        ("display-scale", u32(3)),
        ("unique-chip-id", struct.pack("<Q", PCLM10_ECID)),
        ("chip-id", u32(0x8015)),
        ("certificate-production-status", u32(0)),
        ("effective-production-status-ap", u32(0)),
        ("certificate-security-mode", u32(1)),
        ("effective-security-mode-sep", u32(1)),
    ]
    if owned_memory_size is not None:
        chosen_props.append(("darwinboot-memory-size", u64(owned_memory_size)))

    chosen = node(
        "chosen",
        chosen_props,
        [
            node("memory-map", mmap_props),
            node("asmb", [ph(), ("lp-sip0", u64(0x6f)), ("lp-sip1", u32(1))]),
        ],
    )

    cpu_nodes = []
    for core in range(cpus_count):
        if core < 4:
            clk = 1785600000
        elif core < 7:
            clk = 2419200000
        else:
            clk = 2956800000
        cpu_nodes.append(
            node(
                f"cpu{core}",
                [
                    ph(),
                    ("device_type", cstr("cpu")),
                    ("compatible", cstr("qcom,kryo485") + cstr("ARM,v8")),
                    ("reg", u32(core << 8)),
                    ("cpu-id", u32(core)),
                    ("cluster-id", u32(0)),
                    ("cluster-type", u32(0)),
                    ("die-cluster-id", u32(0)),
                    ("cluster-core-id", u32(core)),
                    ("l2-cache-id", u32(core)),
                    ("l3-cache-id", u32(0)),
                    ("state", cstr("running" if core == 0 else "stopped")),
                    ("timebase-frequency", u32(TIMEBASE)),
                    ("clock-frequency", u32(clk)),
                    ("bus-frequency", u32(100000000)),
                    ("memory-frequency", u32(2133000000)),
                    ("peripheral-frequency", u32(TIMEBASE)),
                    ("fixed-frequency", u32(TIMEBASE)),
                ],
            )
        )
    cpus = node("cpus", [ph(), ("#address-cells", u32(1)), ("#size-cells", u32(0))], cpu_nodes)

    gic = node(
        "gic",
        [
            ph(),
            ("compatible", cstr("arm,gic-v3")),
            ("interrupt-controller", b""),
            ("virtual-timer-intid", u32(VTIMER_INTID)),
            ("reg", u64(GICD - SOC_BASE, GICD_SIZE, GICR - SOC_BASE, GICR_SIZE)),
        ],
    )
    arm_io = node(
        "arm-io",
        [
            ph(),
            ("device_type", cstr("sweet-io")),
            ("compatible", cstr("sweet-io") + cstr("arm-io")),
            ("#address-cells", u32(2)),
            ("#size-cells", u32(2)),
            ("ranges", u64(0, SOC_BASE, 0x100000000 - SOC_BASE)),
        ],
        [
            gic,
            node("wdt", [ph(), ("no-pmu", u32(1))]),
            node(
                "disp0",
                [
                    ph(),
                    ("device_type", cstr("display-subsystem")),
                    ("compatible", cstr("sweet,display")),
                    ("canvas-width", u32(1125)),
                    ("canvas-height", u32(2436)),
                ],
            ),
        ],
    )

    fbcon = node(
        "sweet-fbcon",
        [
            ph(),
            ("reg", u64(FB, FB_SIZE)),
            ("width", u32(FB_W)),
            ("height", u32(FB_H)),
            ("stride", u32(FB_STRIDE)),
        ],
    )
    ufs = node(
        "sweet-ufs",
        [
            ph(),
            ("compatible", cstr("qcom,ufshc")),
            ("reg", u64(UFS, UFS_SIZE)),
        ],
    )
    defaults = node("defaults", [ph(), ("no-effaceable-storage", u32(1))])
    buttons = node(
        "buttons",
        [
            ph(),
            ("button-names", cstr("volup") + cstr("voldown") + cstr("hold")),
            ("home-button-type", u32(2)),
            ("opposed-power-vol-buttons", u32(1)),
        ],
    )
    product_props = d22ap_product(d22ap_adt)
    product = [node("product", [ph()] + product_props)] if product_props else []
    pram = node("pram", [ph(), ("reg", u64(PANIC_LOG_BASE, PANIC_LOG_SIZE))])

    return node(
        "device-tree",
        [
            ph(),
            (
                "compatible",
                cstr("D22AP")
                + cstr("iPhone10,3")
                + cstr("oppo,pclm10")
                + cstr("qcom,sm8150")
                + cstr("AppleARM"),
            ),
            ("model", cstr("iPhone10,3")),
            ("target-type", cstr("D22")),
            ("target-sub-type", cstr("D22AP")),
            ("manufacturer", cstr("Apple Inc.")),
            ("#address-cells", u32(2)),
            ("#size-cells", u32(2)),
        ],
        [chosen, defaults, cpus, arm_io, fbcon, buttons, pram] + product + [ufs],
    )


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("out", type=Path)
    p.add_argument("--d22ap-adt", type=Path, default=SCRIPTS_DIR.parent / "sweet_work" / "d22ap.adt")
    p.add_argument("--owned-memory-size", type=lambda s: int(s, 0), default=0x10000000)
    p.add_argument("--ramdisk", action="store_true", help="Include empty /chosen/memory-map/RAMDisk placeholder")
    p.add_argument("--cpus", type=int, default=8)
    args = p.parse_args()

    root = build(
        d22ap_adt=args.d22ap_adt if args.d22ap_adt.is_file() else None,
        owned_memory_size=args.owned_memory_size,
        has_ramdisk=args.ramdisk,
        cpus_count=args.cpus,
    )
    blob = adt.serialize(root)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_bytes(blob)
    print(f"{args.out}: {len(blob)} bytes (ramdisk={args.ramdisk}, mem={args.owned_memory_size:#x})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
