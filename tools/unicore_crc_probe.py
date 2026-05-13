#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

SYNC = bytes((0xAA, 0x44, 0xB5))
HEADER_LEN = 24
CRC_LEN = 4


def crc32_reflected(data: bytes, init: int, xorout: int) -> int:
    crc = init & 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            lsb = crc & 1
            crc >>= 1
            if lsb:
                crc ^= 0xEDB88320
        crc &= 0xFFFFFFFF
    return (crc ^ xorout) & 0xFFFFFFFF


def crc32_unreflected(data: bytes, init: int, xorout: int) -> int:
    crc = init & 0xFFFFFFFF
    for byte in data:
        crc ^= (byte << 24) & 0xFFFFFFFF
        for _ in range(8):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return (crc ^ xorout) & 0xFFFFFFFF


def extract_frame(data: bytes, offset: Optional[int] = None) -> Tuple[int, bytes]:
    sync_offset = data.find(SYNC) if offset is None else offset
    if sync_offset < 0:
        raise ValueError("No Unicore binary sync AA 44 B5 found")
    if sync_offset + HEADER_LEN + CRC_LEN > len(data):
        raise ValueError("Not enough bytes after sync to read a full N4 header")

    header = data[sync_offset:sync_offset + HEADER_LEN]
    payload_len = int.from_bytes(header[6:8], "little", signed=False)
    frame_len = HEADER_LEN + payload_len + CRC_LEN
    if sync_offset + frame_len > len(data):
        raise ValueError(
            f"Frame truncated at offset {sync_offset}: need {frame_len} bytes, only {len(data) - sync_offset} available"
        )
    return sync_offset, data[sync_offset:sync_offset + frame_len]


def crc_variants(frame: bytes) -> Dict[str, bytes]:
    return {
        "sync+header+payload": frame[:-4],
        "header+payload_without_sync": frame[3:-4],
        "payload_only": frame[HEADER_LEN:-4],
        "header_only_with_sync": frame[:HEADER_LEN],
        "header_only_without_sync": frame[3:HEADER_LEN],
    }


def compute_all_variants(frame: bytes) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    expected_le = int.from_bytes(frame[-4:], "little", signed=False)
    expected_be = int.from_bytes(frame[-4:], "big", signed=False)

    for scope_name, payload in crc_variants(frame).items():
        for reflected in (True, False):
            for init in (0x00000000, 0xFFFFFFFF):
                for xorout in (0x00000000, 0xFFFFFFFF):
                    if reflected:
                        value = crc32_reflected(payload, init, xorout)
                    else:
                        value = crc32_unreflected(payload, init, xorout)
                    rows.append(
                        {
                            "scope": scope_name,
                            "reflected": reflected,
                            "init": init,
                            "xorout": xorout,
                            "value": value,
                            "matches_little_endian": value == expected_le,
                            "matches_big_endian": value == expected_be,
                        }
                    )
    return rows


def format_variant(row: Dict[str, object]) -> str:
    endian = []
    if row["matches_little_endian"]:
        endian.append("expected_le")
    if row["matches_big_endian"]:
        endian.append("expected_be")
    endian_text = ",".join(endian) if endian else "no-match"
    return (
        f"{row['scope']:<28s} reflected={str(row['reflected']).lower():5s} "
        f"init=0x{int(row['init']):08x} xorout=0x{int(row['xorout']):08x} "
        f"crc=0x{int(row['value']):08x} {endian_text}"
    )


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Probe which CRC32 variant matches a captured Unicore N4 binary frame."
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input", help="Path to a raw capture file, e.g. /tmp/unicore-hybrid.bin")
    source.add_argument("--hex", dest="hex_bytes", help="Raw frame/capture bytes as a hex string")
    parser.add_argument("--offset", type=int, help="Optional byte offset of the frame sync in the input")
    parser.add_argument("--json", help="Optional path to write the probe results as JSON")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.input:
        data = Path(args.input).read_bytes()
    else:
        data = bytes.fromhex(args.hex_bytes)

    sync_offset, frame = extract_frame(data, args.offset)
    msg_id = int.from_bytes(frame[4:6], "little", signed=False)
    payload_len = int.from_bytes(frame[6:8], "little", signed=False)
    expected_le = int.from_bytes(frame[-4:], "little", signed=False)
    expected_be = int.from_bytes(frame[-4:], "big", signed=False)
    rows = compute_all_variants(frame)
    matches = [row for row in rows if row["matches_little_endian"] or row["matches_big_endian"]]

    summary = {
        "sync_offset": sync_offset,
        "message_id": msg_id,
        "payload_length": payload_len,
        "frame_length": len(frame),
        "header_length": HEADER_LEN,
        "expected_crc_little_endian": expected_le,
        "expected_crc_big_endian": expected_be,
        "matches": matches,
        "all_variants": rows,
    }

    print(f"sync_offset={sync_offset} msg_id={msg_id} payload_len={payload_len} frame_len={len(frame)}")
    print(f"expected_crc_le=0x{expected_le:08x} expected_crc_be=0x{expected_be:08x}")
    if matches:
        print("matching_variants:")
        for row in matches:
            print(f"  {format_variant(row)}")
    else:
        print("matching_variants: none")
    print("all_variants:")
    for row in rows:
        print(f"  {format_variant(row)}")

    if args.json:
        Path(args.json).write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")

    return 0 if matches else 1


if __name__ == "__main__":
    raise SystemExit(main())
