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

LC_SEGMENT_64 = 0x19
LC_UNIXTHREAD = 0x5
LC_DYLD_CHAINED_FIXUPS = 0x80000034


def parse_macho(buf: bytearray | bytes) -> dict:
    magic, cputype, cpusub, _ftype, ncmds, _sizeofcmds, _flags, _ = struct.unpack_from("<8I", buf, 0)
    if magic != 0xFEEDFACF:
        raise SystemExit(f"expected 64-bit Mach-O kernel, got magic=0x{magic:x}")
    segs = []
    entry = None
    fixups = None
    off = 32
    for _ in range(ncmds):
        cmd, size = struct.unpack_from("<II", buf, off)
        if cmd == LC_SEGMENT_64:
            name = buf[off + 8 : off + 24].split(b"\0")[0].decode("ascii", errors="replace")
            vmaddr, vmsize, fileoff, filesize = struct.unpack_from("<4Q", buf, off + 24)
            segs.append(
                {
                    "name": name,
                    "vmaddr": vmaddr,
                    "vmsize": vmsize,
                    "fileoff": fileoff,
                    "filesize": filesize,
                }
            )
        elif cmd == LC_UNIXTHREAD:
            entry = struct.unpack_from("<Q", buf, off + 16 + 32 * 8)[0]
        elif cmd == LC_DYLD_CHAINED_FIXUPS:
            fixups = struct.unpack_from("<II", buf, off + 8)
        off += size
    return {"cputype": cputype, "cpusub": cpusub & 0xFF, "segs": segs, "entry": entry, "fixups": fixups}


def decode_chained_ptr(raw: int, fmt: int, image_base: int) -> tuple[int, int, int]:
    """Return (unslid target VA, next delta, stride) for one LC_DYLD_CHAINED_FIXUPS entry."""
    if fmt in (2, 6):  # DYLD_CHAINED_PTR_64 / DYLD_CHAINED_PTR_64_OFFSET
        if raw >> 63:
            raise SystemExit("unexpected bind in kernel chained fixups")
        target = raw & ((1 << 36) - 1)
        high8 = (raw >> 36) & 0xFF
        nxt = (raw >> 51) & 0xFFF
        if fmt == 6:
            target += image_base
        return target | (high8 << 56), nxt, 4
    if fmt == 8:  # DYLD_CHAINED_PTR_64_KERNEL_CACHE
        target = raw & ((1 << 30) - 1)
        nxt = (raw >> 51) & 0xFFF
        return image_base + target, nxt, 4
    if fmt in (1, 7, 9, 12):
        auth = raw >> 63
        nxt = (raw >> 51) & 0x7FF
        stride = 4 if fmt in (7, 12) else 8
        if auth:
            target = raw & 0xFFFFFFFF
        else:
            target = raw & ((1 << 43) - 1)
            target |= ((raw >> 43) & 0xFF) << 56
        if fmt in (7, 9, 12) or auth:
            target += image_base
        return target, nxt, stride
    raise SystemExit(f"unsupported chained pointer format {fmt}")


def apply_chained_fixups(buf: bytearray, macho: dict) -> int:
    """Rewrite LC_DYLD_CHAINED_FIXUPS chains in-place to plain unslid kernel VAs for DarwinBoot."""
    if not macho["fixups"]:
        return 0
    data_off, _ = macho["fixups"]
    _, starts_off, _, _, imports_count, _, _ = struct.unpack_from("<7I", buf, data_off)
    if imports_count != 0:
        raise SystemExit(f"kernel has {imports_count} imports; only rebases supported")
    image_base = macho["segs"][0]["vmaddr"]
    seg_count = struct.unpack_from("<I", buf, data_off + starts_off)[0]
    nfix = 0
    for i in range(seg_count):
        seg_info = struct.unpack_from("<I", buf, data_off + starts_off + 4 + 4 * i)[0]
        if not seg_info:
            continue
        base = data_off + starts_off + seg_info
        _size, page_size, fmt, _seg_offset, _max_valid, page_count = struct.unpack_from("<IHHQIH", buf, base)
        starts = struct.unpack_from(f"<{page_count}H", buf, base + 22)
        seg = macho["segs"][i]
        for p, start in enumerate(starts):
            if start == 0xFFFF:
                continue
            if start & 0x8000:
                raise SystemExit("DYLD_CHAINED_PTR_START_MULTI not supported")
            loc = seg["fileoff"] + p * page_size + start
            while True:
                raw = struct.unpack_from("<Q", buf, loc)[0]
                target, nxt, stride = decode_chained_ptr(raw, fmt, image_base)
                struct.pack_into("<Q", buf, loc, target)
                nfix += 1
                if not nxt:
                    break
                loc += nxt * stride
    return nfix


def write_rebased_kernel(src: Path, dst: Path) -> tuple[int, dict]:
    buf = bytearray(src.read_bytes())
    macho = parse_macho(buf)
    nfix = apply_chained_fixups(buf, macho)
    if macho["fixups"] and nfix <= 0:
        raise SystemExit("LC_DYLD_CHAINED_FIXUPS present but applied 0 fixups")
    dst.write_bytes(buf)
    return nfix, macho


def recompute_bootimg_id(buf: bytearray) -> None:
    pg = struct.unpack_from("<I", buf, 36)[0]
    hdr_ver = struct.unpack_from("<I", buf, 40)[0]
    ksize, _, rsize, _, ssize, _ = struct.unpack_from("<6I", buf, 8)
    rec_size, _, _hdr_size, dtb_size, _ = (
        struct.unpack_from("<IQIIQ", buf, 1632) if hdr_ver >= 1 else (0, 0, 0, 0, 0)
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
    template_payload: bytes | None = None,
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
        rebased_kernel = tmp / "kernel.rebased.macho"
        nfix, macho = write_rebased_kernel(kernel, rebased_kernel)
        text_va = macho["segs"][0]["vmaddr"] if macho["segs"] else 0
        print(
            f"Rebased kernel {kernel.name}: nfix={nfix}, text_va=0x{text_va:x}, entry=0x{(macho['entry'] or 0):x}"
        )
        raw = tmp / "payload.img"
        use_template_fat = template_payload is not None and len(template_payload) == size_bytes
        raw.write_bytes(template_payload if use_template_fat else (b"\x00" * size_bytes))
        dev = (
            subprocess.check_output(
                ["hdiutil", "attach", "-imagekey", "diskimage-class=CRawDiskImage", "-nomount", str(raw)],
                text=True,
            )
            .strip()
            .split()[0]
        )
        try:
            if not use_template_fat:
                subprocess.run(["newfs_msdos", "-F", "32", "-c", "1", "-v", "UNIXNU", dev], check=True)
            mnt = tmp / "mnt"
            mnt.mkdir()
            subprocess.run(["mount", "-t", "msdos", dev, str(mnt)], check=True)
            try:
                if use_template_fat:
                    for item in list(mnt.iterdir()):
                        if item.name.startswith("."):
                            continue
                        if item.is_dir():
                            shutil.rmtree(item)
                        else:
                            item.unlink()
                payloads_dir = mnt / "EFI/UniXNU/Payloads"
                payloads_dir.mkdir(parents=True)
                (mnt / "EFI/UniXNU/unixnu.toml").write_text(toml, encoding="utf-8")
                shutil.copy2(rebased_kernel, payloads_dir / "kernel.macho")
                shutil.copy2(adt, payloads_dir / "kernel.macho.adt")
                if ramdisk:
                    shutil.copy2(ramdisk, payloads_dir / "ramdisk.macho")
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

    # Assemble Android v2 boot.img preserving ABL's verified template header + stock dtb
    tmpl = bytearray(args.template.read_bytes())
    pg = struct.unpack_from("<I", tmpl, 36)[0]
    hdr_ver = struct.unpack_from("<I", tmpl, 40)[0]
    old_ksize, _, old_rsize, _, old_ssize, _ = struct.unpack_from("<6I", tmpl, 8)
    rec_size, _, _hdr_size, dtb_size, _ = struct.unpack_from("<IQIIQ", tmpl, 1632)
    align = lambda n: (n + pg - 1) & ~(pg - 1) if n else 0
    old_r_off = pg + align(old_ksize)
    old_payload = bytes(tmpl[old_r_off : old_r_off + old_rsize]) if old_rsize == args.payload_size else None
    old_dtb_off = pg + align(old_ksize) + align(old_rsize) + align(old_ssize) + align(rec_size)
    dtb_bytes = bytes(tmpl[old_dtb_off : old_dtb_off + dtb_size])

    payload_path = args.out_dir / "payload.img"
    build_fat32_payload(
        template_payload=old_payload,
        kernel=args.kernel,
        adt=adt_path,
        ramdisk=args.ramdisk,
        bootargs=args.bootargs,
        size_bytes=args.payload_size,
        out_img=payload_path,
    )

    uefi_k = extract_kernel_from_bootimg(args.uefi_bootimg)
    payload_bytes = payload_path.read_bytes()

    hdr = bytearray(tmpl[:pg])
    struct.pack_into("<I", hdr, 8, len(uefi_k))
    struct.pack_into("<I", hdr, 16, len(payload_bytes))
    struct.pack_into("<I", hdr, 24, 0)
    if hdr_ver >= 2:
        struct.pack_into("<I", hdr, 1648, len(dtb_bytes))

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
