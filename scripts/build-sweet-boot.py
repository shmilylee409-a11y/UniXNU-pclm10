#!/usr/bin/env python3
"""Build a FAT32 UniXNU payload image and Android v2 boot.img for PCLM10 ios-sweet (16K XNU)."""
from __future__ import annotations

import argparse
import hashlib
import shutil
import struct
import subprocess
import tempfile
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent
ROOT = SCRIPTS_DIR.parent


def recompute_bootimg_id(buf: bytearray) -> None:
    pg = struct.unpack_from("<I", buf, 36)[0]
    hdr_ver = struct.unpack_from("<I", buf, 40)[0]
    ksize, _, rsize, _, ssize, _ = struct.unpack_from("<6I", buf, 8)
    rec_size, _, dtb_size, _ = (
        struct.unpack_from("<IQIQ", buf, 1632) if hdr_ver >= 1 else (0, 0, 0, 0)
    )
    if hdr_ver < 2:
        dtb_size = 0

    align = lambda n: (n + pg - 1) & ~(pg - 1) if n else 0
    k_off = pg
    r_off = k_off + align(ksize)
    s_off = r_off + align(rsize)
    rec_off = s_off + align(ssize)
    dtb_off = rec_off + align(rec_size)

    h = hashlib.sha1()
    h.update(buf[k_off : k_off + ksize])
    h.update(struct.pack("<I", ksize))
    h.update(buf[r_off : r_off + rsize])
    h.update(struct.pack("<I", rsize))
    h.update(buf[s_off : s_off + ssize])
    h.update(struct.pack("<I", ssize))
    if hdr_ver >= 1 and rec_size:
        h.update(buf[rec_off : rec_off + rec_size])
        h.update(struct.pack("<I", rec_size))
    if hdr_ver >= 2 and dtb_size:
        h.update(buf[dtb_off : dtb_off + dtb_size])
        h.update(struct.pack("<I", dtb_size))
    buf[576:608] = h.digest() + b"\x00" * 12


def build_fat32_payload(
    *,
    kernel: Path,
    adt: Path,
    ramdisk: Path | None,
    bootargs: str,
    size_bytes: int,
    out_img: Path,
) -> None:
    if size_bytes < 64 * 1024 * 1024 or (size_bytes % 4096) != 0:
        raise SystemExit("payload size must be >= 64 MiB and 4 KiB aligned")

    initrd_line = 'initrd = "Payloads/ramdisk.macho"\n' if ramdisk else ""
    toml = (
        "version = 1\n"
        "[boot]\n"
        "timeout = 1\n"
        'default = "xnu"\n'
        "[kits]\n"
        "autoload = false\n"
        "[[entry]]\n"
        'id = "xnu"\n'
        'title = "OPPO Reno Ace - iOS 16.7.16 (20H392)"\n'
        'type = "xnu"\n'
        'path = "Payloads/kernel.macho"\n'
        f"{initrd_line}"
        f'args = "{bootargs}"\n'
        "[[entry]]\n"
        'id = "reset"\n'
        'title = "Restart"\n'
        'type = "action"\n'
        'action = "reset"\n'
    )

    out_img.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="pclm10-fat-") as td:
        tmp = Path(td)
        raw = tmp / "payload.img"
        raw.write_bytes(b"\x00" * size_bytes)
        dev = (
            subprocess.check_output(
                ["hdiutil", "attach", "-imagekey", "diskimage-class=CRawDiskImage", "-nomount", str(raw)],
                text=True,
            )
            .strip()
            .split()[0]
        )
        try:
            subprocess.run(["newfs_msdos", "-F", "32", "-c", "1", "-v", "UNIXNU", dev], check=True)
            mnt = tmp / "mnt"
            mnt.mkdir()
            subprocess.run(["mount", "-t", "msdos", dev, str(mnt)], check=True)
            try:
                (mnt / "EFI/UniXNU").mkdir(parents=True)
                (mnt / "Payloads").mkdir(parents=True)
                (mnt / "EFI/UniXNU/unixnu.toml").write_text(toml, encoding="utf-8")
                shutil.copy2(kernel, mnt / "Payloads/kernel.macho")
                shutil.copy2(adt, mnt / "Payloads/kernel.macho.adt")
                if ramdisk:
                    shutil.copy2(ramdisk, mnt / "Payloads/ramdisk.macho")
            finally:
                subprocess.run(["umount", str(mnt)], check=True)
        finally:
            subprocess.run(["hdiutil", "detach", dev], check=True)
        shutil.copy2(raw, out_img)


def extract_kernel_from_bootimg(bootimg: Path) -> bytes:
    d = bootimg.read_bytes()
    assert d[:8] == b"ANDROID!", f"{bootimg} is not an Android boot image"
    pg = struct.unpack_from("<I", d, 36)[0]
    ksize = struct.unpack_from("<I", d, 8)[0]
    return d[pg : pg + ksize]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--template", type=Path, default=ROOT / "build_artifacts_run_42935363935/boot_qud_v3_probe0.img")
    p.add_argument("--uefi-bootimg", type=Path, required=True, help="CI boot-uefi-32m.img containing updated SIZE_32MB UEFI+BootShim")
    p.add_argument("--kernel", type=Path, required=True, help="xnu-8796 kernel.development.sweet Mach-O")
    p.add_argument("--d22ap-adt", type=Path, default=ROOT / "sweet_work/d22ap.adt")
    p.add_argument("--ramdisk", type=Path, default=None, help="Optional ramdisk (e.g. sweet_work/ramdisk.macho for rd=md0)")
    p.add_argument("--bootargs", default="debug=0xc serial=3 cpus=1 keepsyms=1 -enable_kprintf_spam rd=md0 sweetusb=2 pclm10-autoreset=45 pclm10-panic-reset=8")
    p.add_argument("--owned-memory-size", type=lambda s: int(s, 0), default=0x18000000)
    p.add_argument("--payload-size", type=int, default=78643200)
    p.add_argument("--out-dir", type=Path, required=True)
    args = p.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    adt_path = args.out_dir / "pclm10-sweet.adt"
    mkadt_cmd = [
        "python3",
        str(SCRIPTS_DIR / "mkadt-sweet-pclm10.py"),
        str(adt_path),
        "--d22ap-adt",
        str(args.d22ap_adt),
        "--owned-memory-size",
        hex(args.owned_memory_size),
    ]
    if args.ramdisk:
        mkadt_cmd.append("--ramdisk")
    subprocess.run(mkadt_cmd, check=True)

    payload_path = args.out_dir / "payload.img"
    build_fat32_payload(
        kernel=args.kernel,
        adt=adt_path,
        ramdisk=args.ramdisk,
        bootargs=args.bootargs,
        size_bytes=args.payload_size,
        out_img=payload_path,
    )

    # Assemble Android v2 boot.img preserving ABL's verified template header + stock dtb
    tmpl = bytearray(args.template.read_bytes())
    pg = struct.unpack_from("<I", tmpl, 36)[0]
    hdr_ver = struct.unpack_from("<I", tmpl, 40)[0]
    old_ksize, _, old_rsize, _, old_ssize, _ = struct.unpack_from("<6I", tmpl, 8)
    rec_size, _, dtb_size, _ = struct.unpack_from("<IQIQ", tmpl, 1632)
    align = lambda n: (n + pg - 1) & ~(pg - 1) if n else 0
    old_dtb_off = pg + align(old_ksize) + align(old_rsize) + align(old_ssize) + align(rec_size)
    dtb_bytes = bytes(tmpl[old_dtb_off : old_dtb_off + dtb_size])

    uefi_k = extract_kernel_from_bootimg(args.uefi_bootimg)
    payload_bytes = payload_path.read_bytes()

    hdr = bytearray(tmpl[:pg])
    struct.pack_into("<I", hdr, 8, len(uefi_k))
    struct.pack_into("<I", hdr, 16, len(payload_bytes))
    struct.pack_into("<I", hdr, 24, 0)
    if hdr_ver >= 2:
        struct.pack_into("<I", hdr, 1644, len(dtb_bytes))

    def pad(b: bytes) -> bytes:
        rem = len(b) % pg
        return b if rem == 0 else b + b"\x00" * (pg - rem)

    out = bytearray(hdr + pad(uefi_k) + pad(payload_bytes) + pad(dtb_bytes))
    recompute_bootimg_id(out)
    boot_out = args.out_dir / "boot-sweet.img"
    boot_out.write_bytes(out)
    print(
        f"Built {boot_out} ({len(out)} bytes, uefi_k={len(uefi_k)}, payload={len(payload_bytes)}, dtb={len(dtb_bytes)}, sha256={hashlib.sha256(out).hexdigest()})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
