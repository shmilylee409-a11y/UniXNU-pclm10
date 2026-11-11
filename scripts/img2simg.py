#!/usr/bin/env python3
"""Convert a raw filesystem image (4 KiB block aligned) into an Android sparse image."""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

SPARSE_MAGIC = 0xED26FF3A
CHUNK_RAW = 0xCAC1
CHUNK_DONT_CARE = 0xCAC3
BLK = 4096
MAX_CHUNK_BLOCKS = 16384  # 64 MiB per chunk


def convert(src: Path, dst: Path) -> None:
    size = src.stat().st_size
    if size % BLK != 0:
        raise SystemExit(f"{src} size ({size}) must be a multiple of {BLK}")
    total_blocks = size // BLK
    zero_block = b"\x00" * BLK

    chunks: list[tuple[int, int, bytes]] = []  # (type, blocks, data)
    with src.open("rb") as f:
        cur_type = None
        cur_blocks = 0
        cur_buf = bytearray()

        def flush():
            nonlocal cur_type, cur_blocks, cur_buf
            if cur_blocks:
                chunks.append((cur_type, cur_blocks, bytes(cur_buf)))
                cur_type = None
                cur_blocks = 0
                cur_buf.clear()

        for _ in range(total_blocks):
            b = f.read(BLK)
            is_zero = b == zero_block
            t = CHUNK_DONT_CARE if is_zero else CHUNK_RAW
            if cur_type != t or cur_blocks >= MAX_CHUNK_BLOCKS:
                flush()
                cur_type = t
            cur_blocks += 1
            if not is_zero:
                cur_buf.extend(b)
        flush()

    hdr = struct.pack("<IHHHHIIII", SPARSE_MAGIC, 1, 0, 28, 12, BLK, total_blocks, len(chunks), 0)
    with dst.open("wb") as out:
        out.write(hdr)
        for t, blocks, data in chunks:
            sz = 12 + (len(data) if t == CHUNK_RAW else 0)
            out.write(struct.pack("<HHII", t, 0, blocks, sz))
            if t == CHUNK_RAW:
                out.write(data)
    print(f"{dst}: {dst.stat().st_size} bytes sparse ({total_blocks} blocks, {len(chunks)} chunks)")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("src", type=Path)
    p.add_argument("dst", type=Path)
    args = p.parse_args()
    convert(args.src, args.dst)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
