"""
Parse MEXE02 serial dump - extract present position from '99 00 82' monitoring packets.

Structure of a '99 00 82' Read packet:
  Header (7 bytes):  ff 01 00 05 99 00 82
  Block-0 data (28 bytes) starts at byte 7:
    bytes  7-10: present position   (LE int32, unit = 0.001 mm)
    bytes 11-14: command position   (LE int32, unit = 0.001 mm)
  Block-0 tick counter (4 bytes): bytes 36-39  (BE uint32 - free-running timer)
  Sub-blocks 1-4 follow with more parameters.

The 4-byte BE tick counter is used for sub-second timing.
"""
import re
import struct
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import numpy as np

FILE1 = (r"C:\Users\thana\01_Fang\Advance Practice\Yuzu_code-Image-processing-motor-"
         r"\motion_cmd\dump_acc.txt")
FILE2 = (r"C:\Users\thana\01_Fang\Advance Practice\Yuzu_code-Image-processing-motor-"
         r"\motion_cmd\dump_acc_2.txt")


# ── Low-level parser ─────────────────────────────────────────────────────────

def iter_packets(path):
    """Yield (datetime, direction, bytearray) for each packet in the dump."""
    ts = None
    direction = None
    buf = bytearray()

    ts_re  = re.compile(r'\[(\d{2}/\d{2}/\d{4} \d{2}:\d{2}:\d{2})\] (Read|Written) data')
    hex_re = re.compile(r'^[\s\d]+([0-9a-f]{2}(?:\s+[0-9a-f]{2})*)', re.IGNORECASE)

    with open(path, encoding='utf-8', errors='ignore') as f:
        for line in f:
            m = ts_re.search(line)
            if m:
                if ts is not None and buf:
                    yield ts, direction, bytes(buf)
                ts = datetime.strptime(m.group(1), '%d/%m/%Y %H:%M:%S')
                direction = m.group(2)
                buf = bytearray()
                continue
            stripped = line.strip()
            if not stripped:
                continue
            # strip trailing ASCII column (3+ spaces then printable)
            part = re.split(r'\s{3,}', stripped)[0]
            for tok in part.split():
                if len(tok) == 2:
                    try:
                        buf.append(int(tok, 16))
                    except ValueError:
                        pass
    if ts is not None and buf:
        yield ts, direction, bytes(buf)


def is_pos_packet(data):
    """True if this is the '99 00 82' monitoring packet."""
    return (len(data) >= 40 and
            data[0] == 0xff and data[1] == 0x01 and
            data[4] == 0x99 and data[5] == 0x00 and data[6] == 0x82)


def extract_pos_record(ts, data):
    """
    Returns (tick_u32, datetime, pos_mm, cmd_mm) from a '99 00 82' packet.
    tick_u32: big-endian 32-bit counter at bytes 36-39 (block-0 end marker).
    """
    pos_raw = struct.unpack_from('<i', data, 7)[0]   # signed LE int32
    cmd_raw = struct.unpack_from('<i', data, 11)[0]  # command pos
    tick    = struct.unpack_from('>I', data, 36)[0]  # BE uint32 tick
    return tick, ts, pos_raw * 0.001, cmd_raw * 0.001


# ── Time reconstruction from tick counters ────────────────────────────────────

def reconstruct_time(records):
    """
    Convert (tick, datetime, pos, cmd) records to (t_seconds, pos_mm) arrays.

    Calibration:
      freq = total tick span / total wall-clock span (handles 32-bit wrap-around).
    Then:
      t[i] = cumulative_ticks[i] / freq
    """
    if not records:
        return np.array([]), np.array([])

    ticks = np.array([r[0] for r in records], dtype=np.int64)
    walls = np.array([(r[1] - records[0][1]).total_seconds() for r in records])
    pos   = np.array([r[2] for r in records])

    # Wrap-around-safe cumulative tick sum
    dtick = np.diff(ticks)
    dtick[dtick < 0] += 2**32          # handle 32-bit rollover
    cum_ticks = np.zeros(len(ticks), dtype=np.float64)
    cum_ticks[1:] = np.cumsum(dtick)

    # Calibrate: use total wall-clock duration
    total_wall = walls[-1] - walls[0]
    total_tick = cum_ticks[-1]
    freq = total_tick / total_wall if total_wall > 0 else 1.0

    t_sec = cum_ticks / freq
    return t_sec, pos


# ── High-level extraction ─────────────────────────────────────────────────────

def parse_file(path):
    records = []
    for ts, direction, data in iter_packets(path):
        if direction == 'Read' and is_pos_packet(data):
            records.append(extract_pos_record(ts, data))
    return records


# ── Motion detection ──────────────────────────────────────────────────────────

def detect_ops(t, pos, min_dist=0.5, gap_thresh=1.5):
    """Return list of (i_start, i_end) index pairs for each motion segment."""
    if len(t) < 10:
        return []

    vel = np.gradient(pos, t)
    moving = np.abs(vel) > 0.05   # mm/s threshold

    segs = []
    in_seg = False
    s = 0
    for i, mv in enumerate(moving):
        if mv and not in_seg:
            s = i
            in_seg = True
        elif not mv and in_seg:
            if abs(pos[i] - pos[s]) >= min_dist:
                segs.append((s, i))
            in_seg = False
    if in_seg and abs(pos[-1] - pos[s]) >= min_dist:
        segs.append((s, len(t) - 1))

    # Merge closely spaced segments
    merged = []
    if segs:
        cs, ce = segs[0]
        for ns, ne in segs[1:]:
            if t[ns] - t[ce] < gap_thresh:
                ce = ne
            else:
                merged.append((cs, ce))
                cs, ne = ns, ne
                ce = ne
        merged.append((cs, ce))

    return merged


# ── Velocity profile analysis ─────────────────────────────────────────────────

def analyze_op(t, pos, i0, i1, label):
    t_op  = t[i0:i1+1]
    p_op  = pos[i0:i1+1]
    dt    = np.diff(t_op)
    dt[dt == 0] = 1e-4
    vel   = np.diff(p_op) / dt

    p_start = p_op[0]
    p_end   = p_op[-1]
    dur     = t_op[-1] - t_op[0]
    dist    = p_end - p_start
    peak_v  = float(np.percentile(np.abs(vel), 98))
    avg_v   = float(np.mean(np.abs(vel[np.abs(vel) > 0.05]))) if np.any(np.abs(vel) > 0.05) else 0

    print(f"  {label}")
    print(f"    start={t_op[0]:.2f}s  end={t_op[-1]:.2f}s  dur={dur:.2f}s")
    print(f"    pos:  {p_start:.3f}mm -> {p_end:.3f}mm  (d={dist:+.3f}mm)")
    print(f"    vel:  peak={peak_v:.3f}mm/s  avg={avg_v:.3f}mm/s")
    return t_op, p_op, vel, dur, dist, peak_v


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_file(path, title, op_labels, color, axs_full, ax_idx):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")

    records = parse_file(path)
    print(f"  Position packets found: {len(records)}")
    if not records:
        print("  ERROR: no '99 00 82' packets detected.")
        return None

    t, pos = reconstruct_time(records)
    print(f"  Time span: {t[-1]:.1f} s  ({len(t)} samples)")
    print(f"  Position range: {pos.min():.3f} .. {pos.max():.3f} mm")

    ops = detect_ops(t, pos)
    print(f"  Detected {len(ops)} motion segment(s):")

    ax = axs_full[ax_idx]
    ax.plot(t, pos, color=color, linewidth=0.9, alpha=0.85)
    ax.axhline(1.0,  color='green',  linestyle='--', lw=0.8, alpha=0.5)
    ax.axhline(29.0, color='orange', linestyle='--', lw=0.8, alpha=0.5)
    ax.set_ylabel('Position (mm)', fontsize=9)
    ax.set_xlabel('Time (s)', fontsize=9)
    ax.set_title(title, fontsize=10)
    ax.grid(True, alpha=0.25)

    op_data = []
    for k, (i0, i1) in enumerate(ops):
        lbl = op_labels[k] if k < len(op_labels) else f"Op{k+1}"
        res = analyze_op(t, pos, i0, i1, lbl)
        op_data.append(res)

        t_op, p_op = res[0], res[1]
        span_color = f"C{k}"
        ax.axvspan(t_op[0], t_op[-1], alpha=0.08, color=span_color)
        mid_t = (t_op[0] + t_op[-1]) / 2
        mid_p = np.median(p_op)
        ax.text(mid_t, mid_p, lbl, fontsize=7, ha='center', va='center',
                bbox=dict(fc='white', alpha=0.7, boxstyle='round,pad=0.2'))

    return t, pos, ops, op_data


def plot_individual_ops(t, pos, ops, op_labels, title, out_path):
    n = len(ops)
    if n == 0:
        return
    fig, axes = plt.subplots(1, n, figsize=(5*n, 4))
    if n == 1:
        axes = [axes]
    fig.suptitle(title, fontsize=12, fontweight='bold')

    for k, (i0, i1) in enumerate(ops):
        lbl  = op_labels[k] if k < len(op_labels) else f"Op{k+1}"
        pad_pts = max(20, (i1-i0)//5)
        s = max(0, i0-pad_pts)
        e = min(len(t)-1, i1+pad_pts)

        t_op  = t[s:e+1] - t[i0]
        p_op  = pos[s:e+1]
        dt    = np.diff(t[s:e+1])
        dt[dt == 0] = 1e-4
        vel_op = np.diff(pos[s:e+1]) / dt
        t_vel  = (t[s:e+1][:-1] + t[s:e+1][1:]) / 2 - t[i0]

        ax1 = axes[k]
        ax2 = ax1.twinx()
        ax1.plot(t_op, p_op,  color='steelblue', linewidth=1.2, label='pos')
        ax2.plot(t_vel, vel_op, color='tomato', linewidth=0.8, alpha=0.7, label='vel')
        ax2.axhline(0, color='gray', lw=0.5)
        ax1.set_title(lbl, fontsize=8)
        ax1.set_xlabel('Time from start (s)', fontsize=8)
        ax1.set_ylabel('Position (mm)', fontsize=8, color='steelblue')
        ax2.set_ylabel('Velocity (mm/s)', fontsize=8, color='tomato')
        ax1.grid(True, alpha=0.2)

    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.show()
    print(f"Saved: {out_path}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ops1_labels = [
        "Op1  acc=0.1  dec=1.0",
        "Op2  acc=0.5  dec=1.0",
        "Op3  acc=1.0  dec=0.1",
        "Op4  acc=1.0  dec=0.5",
    ]
    ops2_labels = [
        "Op1  acc=0.777",
        "Op2  acc=0.444",
        "Op3  dec=0.888",
        "Op4  dec=0.653",
    ]

    fig, axs = plt.subplots(2, 1, figsize=(16, 9))
    fig.suptitle('MEXE02 Present Position  (unit: 0.001 mm)', fontsize=13, fontweight='bold')

    r1 = plot_file(FILE1, "Dump 1 — varying acc/dec  (spd=1 mm/s)", ops1_labels, 'steelblue', axs, 0)
    r2 = plot_file(FILE2, "Dump 2 — acc=0.777/0.444, dec=0.888/0.653  (spd=1 mm/s)", ops2_labels, 'darkorange', axs, 1)

    plt.tight_layout()
    out1 = FILE1.replace('dump_acc.txt', 'motion_profiles.png')
    plt.savefig(out1, dpi=150)
    plt.show()
    print(f"\nSaved combined plot: {out1}")

    # Individual op plots
    if r1:
        t1, p1, ops1, _ = r1
        plot_individual_ops(t1, p1, ops1, ops1_labels,
                            "Dump 1 — Individual Operations",
                            FILE1.replace('dump_acc.txt', 'dump1_ops.png'))

    if r2:
        t2, p2, ops2, _ = r2
        plot_individual_ops(t2, p2, ops2, ops2_labels,
                            "Dump 2 — Individual Operations",
                            FILE2.replace('dump_acc_2.txt', 'dump2_ops.png'))


if __name__ == '__main__':
    main()
