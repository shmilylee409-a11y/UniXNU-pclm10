#!/usr/bin/env python3
"""Convert a raw filesystem image into an Android sparse image (.simg) skipping zero blocks."""
from __future__ import annotations

import argparse
import struct
from pathlib import Path

SPARSE_MAGIC = 0xED26FF3A
CHUNK_RAW = 0xCAC1
CHUNK_DONT_CARE = 0xCAC3
BLK = 4096


def convert(src: Path, dst: Path) -> None:
    size = src.stat().st_size
    if size % BLK != 0:
        raise SystemExit(f"{src}: size {size} is not {BLK}-byte aligned")
    total_blocks = size // BLK
    zero_blk = b"\x00" * BLK

    chunks: list[tuple[int, int, bytes | None]] = []
    cur_kind: int | None = None
    cur_blocks = 0
    cur_data = bytearray()

    def flush() -> None:
        nonlocal cur_kind, cur_blocks, cur_data
        if cur_kind is not None and cur_blocks > 0:
            chunks.append((cur_kind, cur_blocks, bytes(cur_data) if cur_kind == CHUNK_RAW else None))
        cur_kind = None
        cur_blocks = 0
        cur_data = bytearray()

    with src.open("rb") as f:
        for _ in range(total_blocks):
            blk = f.read(BLK)
            is_zero = (blk == zero_blk)
            kind = CHUNK_DONT_CARE if is_zero else CHUNK_RAW
            if cur_kind != kind or (kind == CHUNK_RAW and len(cur_data) >= 64 * 1024 * 1024):
                flush()
                cur_kind = kind
            cur_blocks += 1
            if kind == CHUNK_RAW:
                cur_data.extend(blk)
    flush()

    with dst.open("wb") as out:
        out.write(struct.pack("<IHHHHIIII", SPARSE_MAGIC, 1, 0, 28, 12, BLK, total_blocks, len(chunks), 0))
        for kind, blocks, data in chunks:
            total_sz = 12 + (len(data) if data is not None else 0)
            out.write(struct.pack("<HHII", kind, 0, blocks, total_sz))
            if data is not None:
                out.write(data)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("src", type=Path)
    p.add_argument("dst", type=Path)
    args = p.parse_args()
    convert(args.src, args.dst)
    print(f"Wrote {args.dst} ({args.dst.stat().st_size >> 20} MiB from {args.src.stat().st_size >> 20} MiB raw)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
