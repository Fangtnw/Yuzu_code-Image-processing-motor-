"""
diff_dump.py — parse MEXE02 serial dump and diff Written blocks.

Usage
-----
  # Analyse one dump (list all Written blocks + Python literals):
  python diff_dump.py  path/to/new_dump.txt

  # Diff two dumps to find what changed (e.g. after changing acceleration):
  python diff_dump.py  path/to/baseline.txt  path/to/new_dump.txt

Output
------
  • Every Written block: index, detected frame type, size, first bytes
  • Decoded values for known frame types (position mm, speed mm/s)
  • For two-file mode: byte-level diff between same-type blocks
  • Python bytes literals for every block → paste into replay_frames.py
"""

import re
import struct
import sys
from collections import defaultdict


# ── Known frame signatures (from motor_controller_node.py) ────────────────────

_SYNC_BYTE = 0x06

# Position frame: 320 bytes, pos int32 LE at offset 130, checksums at 159,311,319
_POS_LEN     = 320
_POS_OFFSET  = 130
_POS_CS_IDX  = (159, 311, 319)

# Speed frame: 240 bytes, spd_lo at 142, spd_hi at 144, checksums at 159,229,239
_SPD_LEN     = 240
_SPD_LO_IDX  = 142
_SPD_HI_IDX  = 144
_SPD_CS_IDX  = (159, 229, 239)

# Execute / Home frames: 40 bytes each, start with ff 03
_EXEC_LEN = 40


def _identify(data: bytes) -> str:
    if len(data) == 1 and data[0] == _SYNC_BYTE:
        return "SYNC"
    if len(data) == _POS_LEN and data[0] == 0xff and data[1] == 0x01:
        return "POSITION_FRAME"
    if len(data) == _SPD_LEN and data[0] == 0xff and data[1] == 0x01:
        return "SPEED_FRAME"
    if len(data) == _EXEC_LEN and data[0] == 0xff and data[1] == 0x03:
        return "EXEC_FRAME"
    return f"UNKNOWN_{len(data)}B"


def _decode(label: str, data: bytes) -> str:
    if label == "POSITION_FRAME":
        pos_um = struct.unpack_from("<i", data, _POS_OFFSET)[0]
        return f"position = {pos_um / 1000:.3f} mm  ({pos_um} µm)"
    if label == "SPEED_FRAME":
        lo = struct.unpack_from("<H", data, _SPD_LO_IDX)[0]
        hi = struct.unpack_from("<H", data, _SPD_HI_IDX)[0]
        spd_ums = lo | (hi << 16)
        return f"speed = {spd_ums / 1000:.3f} mm/s  ({spd_ums} µm/s)"
    return ""


# ── Dump parser ────────────────────────────────────────────────────────────────

def iter_written_blocks(path: str):
    """Yield (block_index, bytes) for every 'Written data' block in *path*."""
    ts_re  = re.compile(r'\[.*?\] (Read|Written) data', re.IGNORECASE)
    hex_re = re.compile(r'^[\s\d]+([0-9a-f]{2}(?:\s+[0-9a-f]{2})*)', re.IGNORECASE)

    current_dir = None
    buf = bytearray()
    block_idx = 0

    def _flush():
        nonlocal block_idx, buf
        if current_dir == "Written" and buf:
            yield block_idx, bytes(buf)
            block_idx += 1

    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = ts_re.search(line)
            if m:
                yield from _flush()
                current_dir = m.group(1).capitalize()  # "Read" or "Written"
                buf = bytearray()
                continue
            if current_dir != "Written":
                continue
            stripped = line.strip()
            if not stripped:
                continue
            # Strip trailing ASCII column (3+ spaces then non-hex text)
            part = re.split(r'\s{3,}', stripped)[0]
            for tok in part.split():
                if len(tok) == 2:
                    try:
                        buf.append(int(tok, 16))
                    except ValueError:
                        pass
    yield from _flush()


# ── Pretty printer ─────────────────────────────────────────────────────────────

def _hex_preview(data: bytes, n: int = 12) -> str:
    preview = data[:n].hex(" ")
    if len(data) > n:
        preview += " ..."
    return preview


def _python_literal(var_name: str, data: bytes, indent: int = 4) -> str:
    pad = " " * indent
    rows = []
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        rows.append(pad + ",".join(f"0x{b:02x}" for b in chunk) + ",")
    return f"{var_name} = bytes([\n" + "\n".join(rows) + "\n])"


def analyse_file(path: str) -> list:
    """Print a summary of all Written blocks and return them as a list."""
    print(f"\n{'='*70}")
    print(f"  FILE: {path}")
    print(f"{'='*70}")

    blocks = list(iter_written_blocks(path))
    print(f"  Written blocks found: {len(blocks)}\n")

    for idx, data in blocks:
        label = _identify(data)
        decoded = _decode(label, data)
        print(f"  Block #{idx:02d}  {label:<20s}  {len(data):4d} bytes  [{_hex_preview(data)}]")
        if decoded:
            print(f"           → {decoded}")

    return blocks


# ── Diff ───────────────────────────────────────────────────────────────────────

def diff_same_type_blocks(blocks_a: list, blocks_b: list) -> None:
    """
    Pair up blocks of the same type (by label) between two dump files and
    print byte-level differences.  Useful for finding the acceleration register.
    """
    print(f"\n{'='*70}")
    print("  DIFF: baseline vs new dump")
    print(f"{'='*70}")

    # Group by label
    by_type_a: dict[str, list] = defaultdict(list)
    by_type_b: dict[str, list] = defaultdict(list)
    for idx, data in blocks_a:
        by_type_a[_identify(data)].append((idx, data))
    for idx, data in blocks_b:
        by_type_b[_identify(data)].append((idx, data))

    all_types = sorted(set(by_type_a) | set(by_type_b))
    found_any_diff = False

    for label in all_types:
        list_a = by_type_a.get(label, [])
        list_b = by_type_b.get(label, [])
        pairs = min(len(list_a), len(list_b))
        if pairs == 0:
            continue

        for i in range(pairs):
            idx_a, data_a = list_a[i]
            idx_b, data_b = list_b[i]
            if data_a == data_b:
                continue

            found_any_diff = True
            print(f"\n  {label}  (baseline block #{idx_a}  vs  new block #{idx_b})")
            diffs = [(off, data_a[off], data_b[off])
                     for off in range(min(len(data_a), len(data_b)))
                     if data_a[off] != data_b[off]]

            # Try to interpret 4-byte LE int at changed region
            changed_offsets = [off for off, _, _ in diffs]
            for off, old, new in diffs:
                note = ""
                # Check if this is part of a 4-byte int32 LE group
                group_start = off - (off % 4)  # align to 4
                # Only annotate the first byte of a 4-byte group
                if off == changed_offsets[0] or (off - changed_offsets[0]) % 4 == 0:
                    try:
                        v_old = struct.unpack_from("<i", data_a, off)[0]
                        v_new = struct.unpack_from("<i", data_b, off)[0]
                        if v_old != v_new:
                            note = (f"  ← int32 LE: {v_old} -> {v_new}"
                                    f"  (as mm/s² if ×0.001: {v_old/1000:.3f} -> {v_new/1000:.3f})")
                    except struct.error:
                        pass
                cs_tag = "  [CHECKSUM]" if off in _POS_CS_IDX + _SPD_CS_IDX else ""
                print(f"    offset {off:4d}: 0x{old:02x} -> 0x{new:02x}{cs_tag}{note}")

    if not found_any_diff:
        print("  No differences found between matching block types.")


# ── Python literals output ─────────────────────────────────────────────────────

def print_literals(blocks: list, tag: str = "") -> None:
    """Print Python bytes literals for all blocks, ready to paste into replay_frames.py."""
    print(f"\n{'='*70}")
    print(f"  PYTHON LITERALS{' — ' + tag if tag else ''}")
    print(f"  (paste these into replay_frames.py)")
    print(f"{'='*70}\n")

    label_count: dict[str, int] = defaultdict(int)
    for idx, data in blocks:
        label = _identify(data)
        count = label_count[label]
        label_count[label] += 1
        var_name = f"{label}_{count}" if count > 0 else label
        print(_python_literal(var_name, data))
        print()


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(0)

    path_a = sys.argv[1]
    path_b = sys.argv[2] if len(sys.argv) >= 3 else None

    blocks_a = analyse_file(path_a)

    if path_b:
        blocks_b = analyse_file(path_b)
        diff_same_type_blocks(blocks_a, blocks_b)
        print_literals(blocks_b, tag="new dump")
    else:
        print_literals(blocks_a)


if __name__ == "__main__":
    main()
