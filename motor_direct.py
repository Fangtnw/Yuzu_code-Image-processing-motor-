#!/usr/bin/env python3
"""
motor_direct.py — standalone DR28T motor controller (no ROS2 required).

Communicates directly with the AZD-KEP driver over USB serial using the same
MEXE02 binary protocol as the ROS2 motor_controller_node.

Usage
─────
    python motor_direct.py                     # auto-detect USB port
    python motor_direct.py --port COM3         # Windows explicit port
    python motor_direct.py --port /dev/ttyACM0 # Linux explicit port
    python motor_direct.py --port COM3 --baud 19200

Prompt commands
───────────────
    m <pos_mm> <speed_mms> [w | <wait_s>]
        Absolute move.
          No 3rd arg  — fire and return immediately (heartbeat keeps motor alive)
          w           — wait for auto-estimated travel time, then return
          <number>    — wait exactly that many seconds, then return

    h                 home (ZHOME)
    dwell <seconds>   pause (no motor command, just waits)
    set accel <val>   set hardware acceleration ramp (mm/s²) — sent to driver
    set decel <val>   set hardware deceleration ramp (mm/s²) — sent to driver
    s                 show status (port, heartbeat, last command)
    ?                 show this help
    q  (or Ctrl-C)    quit

Dependencies
────────────
    pip install pyserial
"""

import argparse
import math
import struct
import sys
import threading
import time
from typing import List, Optional

import serial
import serial.tools.list_ports


# ── ANSI helpers ───────────────────────────────────────────────────────────────

def _c(code: str, text: str) -> str:
    """Wrap *text* in an ANSI colour code (silently skipped on plain terminals)."""
    if not sys.stdout.isatty():
        return text
    codes = {"red": "31", "green": "32", "yellow": "33", "cyan": "36",
             "gray": "90", "bold": "1", "magenta": "35"}
    return f"\033[{codes.get(code, '0')}m{text}\033[0m"


def _ok(msg: str)   -> None: print(_c("green",   f"  [✓] {msg}"))
def _info(msg: str) -> None: print(_c("cyan",    f"  [i] {msg}"))
def _warn(msg: str) -> None: print(_c("yellow",  f"  [!] {msg}"))
def _err(msg: str)  -> None: print(_c("red",     f"  [✗] {msg}"))


# ══════════════════════════════════════════════════════════════════════════════
# Protocol constants — identical to motor_controller_node.py
# (MEXE02 v4 captures, AZD-KEP, COM3, 19200 8E1)
# ══════════════════════════════════════════════════════════════════════════════

_SYNC = bytes([0x06])  # ENQ — driver acknowledges with 0x86

# ── Position frame (320 bytes) ─────────────────────────────────────────────────
# Bulk register-write for data slot 0.  Position (int32 LE, µm) at offset 130.
# Checksums (XOR) at offsets 159, 311, 319.
_POS_FRAME = bytes([
    0xff,0x01,0x00,0x00,0xfc,0x00,0xc3,0x01,0x00,0x2a,0x02,0x00,0x00,0x00,0x20,0x2a,  # 000–015
    0x02,0x00,0x00,0x00,0x40,0x2a,0x02,0x00,0x00,0x00,0x60,0x2a,0x02,0x00,0x00,0x00,  # 016–031
    0x80,0x2a,0x02,0x00,0x00,0x00,0x00,0x68,0xff,0x00,0x00,0x00,0x00,0x00,0xa0,0x2a,  # 032–047
    0x02,0x00,0x00,0x00,0xc0,0x2a,0x02,0x00,0x00,0x00,0xe0,0x2a,0x02,0x00,0x00,0x00,  # 048–063
    0x00,0x2b,0x02,0x00,0x00,0x00,0x20,0x2b,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x77,  # 064–079
    0xff,0x00,0x00,0x00,0x40,0x2b,0x02,0x00,0x00,0x00,0x60,0x2b,0x02,0x00,0x00,0x00,  # 080–095
    0x80,0x2b,0x02,0x00,0x00,0x00,0xa0,0x2b,0x02,0x00,0x00,0x00,0xc0,0x2b,0x02,0x00,  # 096–111
    0x00,0x00,0xe0,0x2b,0x00,0x00,0x00,0xdd,0xff,0x00,0x00,0x00,0x02,0x00,0x00,0x00,  # 112–127
    0x01,0x0c,0xa8,0x61,0x00,0x00,0x21,0x0c,0x00,0x00,0x00,0x00,0x41,0x0c,0x00,0x00,  # 128–143  ← pos int32 [130:134]
    0x00,0x00,0x61,0x0c,0x00,0x00,0x00,0x00,0x81,0x0c,0x00,0x00,0x00,0x00,0x00,0xb9,  # 144–159  ← cs [159]
    0xff,0x00,0x00,0x00,0x00,0x00,0xa1,0x0c,0x00,0x00,0x00,0x00,0xc1,0x0c,0x00,0x00,  # 160–175
    0x00,0x00,0xe1,0x0c,0x00,0x00,0x00,0x00,0x01,0x0d,0x00,0x00,0x00,0x00,0x21,0x0d,  # 176–191
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x52,0xff,0x00,0x00,0x00,0x41,0x0d,0x00,0x00,  # 192–207
    0x00,0x00,0x61,0x0d,0x00,0x00,0x00,0x00,0x81,0x0d,0x00,0x00,0x00,0x00,0xa1,0x0d,  # 208–223
    0x00,0x00,0x00,0x00,0xc1,0x0d,0x00,0x00,0x00,0x00,0xe1,0x0d,0x00,0x00,0x00,0xdf,  # 224–239
    0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x0e,0x00,0x00,0x00,0x00,0x21,0x0e,  # 240–255
    0x00,0x00,0x00,0x00,0x41,0x0e,0x00,0x00,0x00,0x00,0x61,0x0e,0x00,0x00,0x00,0x00,  # 256–271
    0x81,0x0e,0x00,0x00,0x00,0x00,0x00,0x70,0xff,0x02,0x00,0x00,0x00,0x00,0xa1,0x0e,  # 272–287
    0x00,0x00,0x00,0x00,0xc1,0x0e,0x00,0x00,0x00,0x00,0xe1,0x0e,0x00,0x00,0x00,0x00,  # 288–303
    0x01,0x0f,0x00,0x00,0x00,0x00,0x00,0xf9,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x85,  # 304–319  ← cs [311],[319]
])
_POS_OFFSET  = 130
_POS_BASE_UM = 25_000
_POS_CS_IDX  = (159, 311, 319)

# ── Speed frame (240 bytes) ────────────────────────────────────────────────────
# Bulk register-write for operating speed.  Speed (int32 LE, µm/s):
#   low word  (bytes 0-1) at offset 142
#   high word (bytes 2-3) at offset 144
# Checksums (XOR) at offsets 159, 229, 239.
_SPD_FRAME = bytes([
    0xff,0x01,0x00,0x00,0xba,0x00,0xc3,0x01,0xc1,0x29,0x00,0x00,0x00,0x00,0xe1,0x29,  # 000–015
    0x00,0x00,0x00,0x00,0x01,0x2a,0x00,0x00,0x00,0x00,0x21,0x2a,0x00,0x00,0x00,0x00,  # 016–031
    0x41,0x2a,0x00,0x00,0x00,0x00,0x00,0xed,0xff,0x00,0x00,0x00,0x00,0x00,0x61,0x2a,  # 032–047
    0x00,0x00,0x00,0x00,0x81,0x2a,0x00,0x00,0x00,0x00,0xa1,0x2a,0x00,0x00,0x00,0x00,  # 048–063
    0xc1,0x2a,0x00,0x00,0x00,0x00,0xe1,0x2a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xb4,  # 064–079
    0xff,0x00,0x00,0x00,0x01,0x2b,0x00,0x00,0x00,0x00,0x21,0x2b,0x00,0x00,0x00,0x00,  # 080–095
    0x41,0x2b,0x00,0x00,0x00,0x00,0x61,0x2b,0x00,0x00,0x00,0x00,0x81,0x2b,0x00,0x00,  # 096–111
    0x00,0x00,0xa1,0x2b,0x00,0x00,0x00,0xdf,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  # 112–127
    0xc1,0x2b,0x00,0x00,0x00,0x00,0xe1,0x2b,0x00,0x00,0x00,0x00,0x02,0x0c,0xe8,0x03,  # 128–143  ← spd[0:2] at [142:144]
    0x00,0x00,0x22,0x0c,0xe8,0x03,0x00,0x00,0x42,0x0c,0xe8,0x03,0x00,0x00,0x00,0x5a,  # 144–159  ← spd[2:4] at [144:146], cs [159]
    0xff,0x00,0x00,0x00,0x00,0x00,0x62,0x0c,0xe8,0x03,0x00,0x00,0x82,0x0c,0xe8,0x03,  # 160–175
    0x00,0x00,0xa2,0x0c,0xe8,0x03,0x00,0x00,0xc2,0x0c,0xe8,0x03,0x00,0x00,0xe2,0x0c,  # 176–191
    0xe8,0x03,0x00,0x00,0x00,0x00,0x00,0x7a,0xff,0x02,0x00,0x00,0x02,0x0d,0xe8,0x03,  # 192–207
    0x00,0x00,0x22,0x0d,0xe8,0x03,0x00,0x00,0x42,0x0d,0xe8,0x03,0x00,0x00,0x62,0x0d,  # 208–223
    0xe8,0x03,0x00,0x00,0x00,0x58,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xa5,  # 224–239  ← cs [229],[239]
])
_SPD_LO_IDX   = 142
_SPD_HI_IDX   = 144
_SPD_BASE_UMS = 1_000
_SPD_CS_IDX   = (159, 229, 239)

# ── Acceleration frame (240 bytes) ────────────────────────────────────────────
# Register 0x0C03+0x20n (12 data slots).  Unit: 1 nm/s² → multiply mm/s² × 1 000 000.
# Baseline encodes 1 000 000 counts = 1.0 mm/s².  Checksums at 159, 229, 239.
_ACCEL_FRAME = bytes([
    0xff,0x01,0x00,0x00,0xba,0x00,0xc3,0x01,0xc1,0x29,0x00,0x00,0x00,0x00,0xe1,0x29,  # 000-015
    0x00,0x00,0x00,0x00,0x01,0x2a,0x00,0x00,0x00,0x00,0x21,0x2a,0x00,0x00,0x00,0x00,  # 016-031
    0x41,0x2a,0x00,0x00,0x00,0x00,0x00,0xed,0xff,0x00,0x00,0x00,0x00,0x00,0x61,0x2a,  # 032-047
    0x00,0x00,0x00,0x00,0x81,0x2a,0x00,0x00,0x00,0x00,0xa1,0x2a,0x00,0x00,0x00,0x00,  # 048-063
    0xc1,0x2a,0x00,0x00,0x00,0x00,0xe1,0x2a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xb4,  # 064-079
    0xff,0x00,0x00,0x00,0x01,0x2b,0x00,0x00,0x00,0x00,0x21,0x2b,0x00,0x00,0x00,0x00,  # 080-095
    0x41,0x2b,0x00,0x00,0x00,0x00,0x61,0x2b,0x00,0x00,0x00,0x00,0x81,0x2b,0x00,0x00,  # 096-111
    0x00,0x00,0xa1,0x2b,0x00,0x00,0x00,0xdf,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  # 112-127
    0xc1,0x2b,0x00,0x00,0x00,0x00,0xe1,0x2b,0x00,0x00,0x00,0x00,0x03,0x0c,0x40,0x42,  # 128-143  <- DS0 reg 0x0C03
    0x0f,0x00,0x23,0x0c,0x40,0x42,0x0f,0x00,0x43,0x0c,0x40,0x42,0x0f,0x00,0x00,0xbd,  # 144-159  <- CS[159]
    0xff,0x00,0x00,0x00,0x00,0x00,0x63,0x0c,0x40,0x42,0x0f,0x00,0x83,0x0c,0x40,0x42,  # 160-175
    0x0f,0x00,0xa3,0x0c,0x40,0x42,0x0f,0x00,0xc3,0x0c,0x40,0x42,0x0f,0x00,0xe3,0x0c,  # 176-191
    0x40,0x42,0x0f,0x00,0x00,0x00,0x00,0x7a,0xff,0x02,0x00,0x00,0x03,0x0d,0x40,0x42,  # 192-207
    0x0f,0x00,0x23,0x0d,0x40,0x42,0x0f,0x00,0x43,0x0d,0x40,0x42,0x0f,0x00,0x63,0x0d,  # 208-223
    0x40,0x42,0x0f,0x00,0x00,0x58,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x42,  # 224-239  <- CS[229], CS[239]
])
_ACL_LO_IDX   = 142
_ACL_HI_IDX   = 144
_ACL_BASE_RAW = 1_000_000   # nm/s² = 1.0 mm/s²
_ACL_CS_IDX   = (159, 229, 239)

# ── Deceleration frame (240 bytes) ────────────────────────────────────────────
# Register 0x0C04+0x20n.  Same envelope as accel, reg addresses +1.
# Baseline encodes 1 000 000 counts = 1.0 mm/s².  Checksums at 159, 229, 239.
_DECEL_FRAME = bytes([
    0xff,0x01,0x00,0x00,0xba,0x00,0xc3,0x01,0xc1,0x29,0x00,0x00,0x00,0x00,0xe1,0x29,  # 000-015
    0x00,0x00,0x00,0x00,0x01,0x2a,0x00,0x00,0x00,0x00,0x21,0x2a,0x00,0x00,0x00,0x00,  # 016-031
    0x41,0x2a,0x00,0x00,0x00,0x00,0x00,0xed,0xff,0x00,0x00,0x00,0x00,0x00,0x61,0x2a,  # 032-047
    0x00,0x00,0x00,0x00,0x81,0x2a,0x00,0x00,0x00,0x00,0xa1,0x2a,0x00,0x00,0x00,0x00,  # 048-063
    0xc1,0x2a,0x00,0x00,0x00,0x00,0xe1,0x2a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xb4,  # 064-079
    0xff,0x00,0x00,0x00,0x01,0x2b,0x00,0x00,0x00,0x00,0x21,0x2b,0x00,0x00,0x00,0x00,  # 080-095
    0x41,0x2b,0x00,0x00,0x00,0x00,0x61,0x2b,0x00,0x00,0x00,0x00,0x81,0x2b,0x00,0x00,  # 096-111
    0x00,0x00,0xa1,0x2b,0x00,0x00,0x00,0xdf,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  # 112-127
    0xc1,0x2b,0x00,0x00,0x00,0x00,0xe1,0x2b,0x00,0x00,0x00,0x00,0x04,0x0c,0x40,0x42,  # 128-143  <- DS0 reg 0x0C04
    0x0f,0x00,0x24,0x0c,0x40,0x42,0x0f,0x00,0x44,0x0c,0x40,0x42,0x0f,0x00,0x00,0xba,  # 144-159  <- CS[159]
    0xff,0x00,0x00,0x00,0x00,0x00,0x64,0x0c,0x40,0x42,0x0f,0x00,0x84,0x0c,0x40,0x42,  # 160-175
    0x0f,0x00,0xa4,0x0c,0x40,0x42,0x0f,0x00,0xc4,0x0c,0x40,0x42,0x0f,0x00,0xe4,0x0c,  # 176-191
    0x40,0x42,0x0f,0x00,0x00,0x00,0x00,0x7a,0xff,0x02,0x00,0x00,0x04,0x0d,0x40,0x42,  # 192-207
    0x0f,0x00,0x24,0x0d,0x40,0x42,0x0f,0x00,0x44,0x0d,0x40,0x42,0x0f,0x00,0x64,0x0d,  # 208-223
    0x40,0x42,0x0f,0x00,0x00,0x58,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x45,  # 224-239  <- CS[229], CS[239]
])
_DCL_LO_IDX   = 142
_DCL_HI_IDX   = 144
_DCL_BASE_RAW = 1_000_000   # nm/s² = 1.0 mm/s²
_DCL_CS_IDX   = (159, 229, 239)

# ── Execute frames (2 × 40 bytes) ─────────────────────────────────────────────
_EXEC_FRAMES = (
    bytes([
        0xff,0x03,0x00,0x00,0x0c,0x00,0x83,0x00,0x82,0x01,0x01,0x00,0x00,0x00,0x00,0x0d,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,
    ]),
    bytes([
        0xff,0x03,0x00,0x00,0x0c,0x00,0x83,0x00,0x9b,0x01,0x00,0x00,0x00,0x00,0x00,0x15,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,
    ]),
)

# ── Home frames (2 × 40 bytes) ────────────────────────────────────────────────
_HOME_FRAMES = (
    bytes([
        0xff,0x03,0x00,0x00,0x0c,0x00,0x83,0x00,0x82,0x01,0x01,0x00,0x00,0x00,0x00,0x0d,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,
    ]),
    bytes([
        0xff,0x03,0x00,0x00,0x0c,0x00,0x83,0x00,0x9c,0x01,0x01,0x00,0x00,0x00,0x00,0x13,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,
    ]),
)

_HEARTBEAT_FRAME = _EXEC_FRAMES[0]

# ── Hardware limits ────────────────────────────────────────────────────────────
MAX_POS_MM  = 29.0
MAX_SPD_MMS = 100.0
FRAME_DELAY = 0.10   # seconds between frame writes
HB_INTERVAL = 0.20   # heartbeat period

# ── Default hardware accel / decel ────────────────────────────────────────────
# Sent to the driver on every move via _build_accel_frame / _build_decel_frame.
# Override at runtime with  set accel <val>  /  set decel <val>  in the REPL,
# or via the --accel / --decel CLI flags.
DEFAULT_ACCEL_MMS2 = 1.0   # mm/s²
DEFAULT_DECEL_MMS2 = 1.0   # mm/s²


# ── Frame builders ─────────────────────────────────────────────────────────────

def _xor_of(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result


def _build_position_frame(position_mm: float) -> bytes:
    pos_um     = int(round(position_mm * 1_000))
    new_bytes  = struct.pack("<i", pos_um)
    base_bytes = struct.pack("<i", _POS_BASE_UM)
    delta      = _xor_of(base_bytes) ^ _xor_of(new_bytes)
    frame      = bytearray(_POS_FRAME)
    frame[_POS_OFFSET : _POS_OFFSET + 4] = new_bytes
    for idx in _POS_CS_IDX:
        frame[idx] ^= delta
    return bytes(frame)


def _build_speed_frame(speed_mms: float) -> bytes:
    spd_ums    = int(round(speed_mms * 1_000))
    new_bytes  = struct.pack("<i", spd_ums)
    base_bytes = struct.pack("<i", _SPD_BASE_UMS)
    delta      = _xor_of(base_bytes) ^ _xor_of(new_bytes)
    frame      = bytearray(_SPD_FRAME)
    frame[_SPD_LO_IDX : _SPD_LO_IDX + 2] = new_bytes[0:2]
    frame[_SPD_HI_IDX : _SPD_HI_IDX + 2] = new_bytes[2:4]
    for idx in _SPD_CS_IDX:
        frame[idx] ^= delta
    return bytes(frame)


def _build_accel_frame(accel_mms2: float) -> bytes:
    accel_raw  = int(round(accel_mms2 * 1_000_000))
    new_bytes  = struct.pack("<i", accel_raw)
    base_bytes = struct.pack("<i", _ACL_BASE_RAW)
    delta      = _xor_of(base_bytes) ^ _xor_of(new_bytes)
    frame      = bytearray(_ACCEL_FRAME)
    frame[_ACL_LO_IDX : _ACL_LO_IDX + 2] = new_bytes[0:2]
    frame[_ACL_HI_IDX : _ACL_HI_IDX + 2] = new_bytes[2:4]
    for idx in _ACL_CS_IDX:
        frame[idx] ^= delta
    return bytes(frame)


def _build_decel_frame(decel_mms2: float) -> bytes:
    decel_raw  = int(round(decel_mms2 * 1_000_000))
    new_bytes  = struct.pack("<i", decel_raw)
    base_bytes = struct.pack("<i", _DCL_BASE_RAW)
    delta      = _xor_of(base_bytes) ^ _xor_of(new_bytes)
    frame      = bytearray(_DECEL_FRAME)
    frame[_DCL_LO_IDX : _DCL_LO_IDX + 2] = new_bytes[0:2]
    frame[_DCL_HI_IDX : _DCL_HI_IDX + 2] = new_bytes[2:4]
    for idx in _DCL_CS_IDX:
        frame[idx] ^= delta
    return bytes(frame)


def _move_payloads(pos_mm: float, spd_mms: float,
                   accel_mms2: float, decel_mms2: float) -> List[bytes]:
    return [
        _SYNC,
        _build_accel_frame(accel_mms2),
        _build_decel_frame(decel_mms2),
        _build_position_frame(pos_mm),
        _build_speed_frame(spd_mms),
        _SYNC,
        *_EXEC_FRAMES,
    ]


def _home_payloads() -> List[bytes]:
    return [_SYNC, *_HOME_FRAMES]


# ── Travel time estimation ─────────────────────────────────────────────────────

def _travel_time_s(distance_mm: float, speed_mms: float,
                   accel_mms2: float = DEFAULT_ACCEL_MMS2) -> float:
    """
    Estimate travel time using a symmetric trapezoidal velocity profile.

    Two cases:
      Triangular  (short move, never reaches peak speed):
          peak_v = sqrt(distance * accel)
          t      = 2 * peak_v / accel
      Trapezoidal (long move, reaches and holds peak speed):
          t_ramp = speed / accel          (accel + decel phases combined)
          t_flat = (distance - speed²/accel) / speed
          t      = t_ramp + t_flat

    Args:
        distance_mm : Absolute travel distance in mm.
        speed_mms   : Peak (commanded) speed in mm/s.
        accel_mms2  : Assumed acceleration in mm/s².

    Returns:
        Estimated travel time in seconds.
    """
    if distance_mm <= 0.0 or speed_mms <= 0.0 or accel_mms2 <= 0.0:
        return 0.0
    # Distance covered during both accel and decel phases combined
    d_ramp = speed_mms ** 2 / accel_mms2
    if d_ramp >= distance_mm:
        # Triangular profile — motor never reaches peak speed
        peak_v = math.sqrt(distance_mm * accel_mms2)
        return 2.0 * peak_v / accel_mms2
    else:
        # Trapezoidal profile
        t_ramp = speed_mms / accel_mms2
        t_flat = (distance_mm - d_ramp) / speed_mms
        return t_ramp + t_flat


# ══════════════════════════════════════════════════════════════════════════════
# Motor driver (no ROS2)
# ══════════════════════════════════════════════════════════════════════════════

class MotorDriver:
    """
    Manages the serial port and heartbeat for one DR28T axis.

    Usage:
        driver = MotorDriver.open("/dev/ttyACM0")
        driver.move(15.0, 10.0)              # fire and return immediately
        driver.move(15.0, 10.0, wait_s=2.5)  # fire and wait 2.5 s
        driver.home()
        driver.close()
    """

    def __init__(self, ser: serial.Serial) -> None:
        self._ser               = ser
        self._lock              = threading.Lock()
        self._stop_hb           = threading.Event()
        self._hb_thread: Optional[threading.Thread] = None
        self._hb_count          = 0
        self._last_pos_mm: Optional[float] = None   # for distance tracking
        self._last_cmd: Optional[str]       = None

    # ── Factory ───────────────────────────────────────────────────────────────

    @classmethod
    def open(cls, port: str, baudrate: int = 19200) -> "MotorDriver":
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5,
            write_timeout=0.5,
        )
        _ok(f"Opened {port}  ({baudrate} baud, 8E1)")
        return cls(ser)

    # ── Public commands ───────────────────────────────────────────────────────

    def move(self, pos_mm: float, spd_mms: float,
             wait_s: Optional[float] = None,
             accel_mms2: float = DEFAULT_ACCEL_MMS2,
             decel_mms2: float = DEFAULT_DECEL_MMS2) -> None:
        """
        Send an absolute move command, then optionally wait for completion.

        Args:
            pos_mm     : Target absolute position in mm.
            spd_mms    : Operating speed in mm/s.
            wait_s     : Seconds to wait after sending frames.
                         None  → return immediately (heartbeat keeps motor alive).
                         value → block for that many seconds, then return.
            accel_mms2 : Hardware acceleration ramp in mm/s² (sent to driver).
            decel_mms2 : Hardware deceleration ramp in mm/s² (sent to driver).
        """
        distance = abs(pos_mm - (self._last_pos_mm or 0.0))
        est_s    = _travel_time_s(distance, spd_mms, accel_mms2)

        self._last_cmd = (
            f"move pos={pos_mm:.2f} mm  speed={spd_mms:.2f} mm/s  "
            f"accel={accel_mms2:.2f} mm/s²  decel={decel_mms2:.2f} mm/s²"
        )
        self._send(_move_payloads(pos_mm, spd_mms, accel_mms2, decel_mms2))
        self._last_pos_mm = pos_mm

        # Show travel time estimate
        _info(
            f"Est. travel time: {est_s:.2f} s"
            f"  (dist={distance:.1f} mm, accel={accel_mms2:.2f} mm/s²)"
        )

        if wait_s is not None:
            self._wait_with_bar(wait_s)

    def home(self) -> None:
        """Trigger the ZHOME homing sequence."""
        self._last_cmd    = "home (ZHOME)"
        self._last_pos_mm = 0.0
        self._send(_home_payloads())

    def wait(self, duration_s: float) -> None:
        """Block for *duration_s* seconds without sending any command."""
        self._wait_with_bar(duration_s)

    def status(self) -> None:
        """Print current connection and heartbeat status."""
        alive  = self._hb_thread is not None and self._hb_thread.is_alive()
        hb_str = (_c("green", f"running  ({self._hb_count} beats)")
                  if alive else _c("gray", "stopped"))
        _info(f"Port      : {self._ser.port}")
        _info(f"Heartbeat : {hb_str}")
        if self._last_pos_mm is not None:
            _info(f"Last pos  : {self._last_pos_mm:.2f} mm")
        if self._last_cmd:
            _info(f"Last cmd  : {self._last_cmd}")

    def close(self) -> None:
        """Stop heartbeat and close the serial port."""
        self._stop_heartbeat()
        if self._ser.is_open:
            self._ser.close()
        _info("Serial port closed.")

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _send(self, payloads: List[bytes]) -> None:
        """Stop heartbeat → write all frames → restart heartbeat."""
        self._stop_heartbeat()
        t0 = time.monotonic()
        with self._lock:
            for i, frame in enumerate(payloads):
                self._ser.write(frame)
                time.sleep(FRAME_DELAY)
                if self._ser.in_waiting:
                    resp = self._ser.read(self._ser.in_waiting)
                    print(_c("gray", f"      RX[{i}] {resp.hex(' ')}"))
        elapsed = time.monotonic() - t0
        _ok(f"{len(payloads)} frames sent in {elapsed:.2f} s")
        self._start_heartbeat()

    def _wait_with_bar(self, duration_s: float) -> None:
        """Block for *duration_s* showing a live progress bar."""
        BAR = 30
        _info(f"Waiting {duration_s:.2f} s …")
        t_start = time.monotonic()
        t_end   = t_start + duration_s
        while True:
            now     = time.monotonic()
            elapsed = now - t_start
            if elapsed >= duration_s:
                break
            frac    = min(elapsed / duration_s, 1.0)
            filled  = int(frac * BAR)
            bar     = "█" * filled + "░" * (BAR - filled)
            remain  = duration_s - elapsed
            print(
                _c("gray", f"\r  [{bar}] {remain:.1f} s remaining "),
                end="", flush=True
            )
            time.sleep(0.05)
        # Clear the progress line
        print(f"\r{' ' * (BAR + 30)}\r", end="", flush=True)
        _ok(f"Done  (waited {duration_s:.2f} s)")

    def _start_heartbeat(self) -> None:
        self._stop_hb.clear()
        self._hb_count  = 0
        self._hb_thread = threading.Thread(
            target=self._hb_loop, daemon=True, name="dr28t-heartbeat"
        )
        self._hb_thread.start()
        _info("Heartbeat running  (motor stays active)")

    def _stop_heartbeat(self) -> None:
        if self._hb_thread and self._hb_thread.is_alive():
            self._stop_hb.set()
            self._hb_thread.join(timeout=2.0)
        self._hb_thread = None

    def _hb_loop(self) -> None:
        while not self._stop_hb.is_set():
            with self._lock:
                try:
                    self._ser.write(_HEARTBEAT_FRAME)
                    self._hb_count += 1
                except serial.SerialException as exc:
                    _err(f"Heartbeat serial error: {exc}")
                    break
            self._stop_hb.wait(timeout=HB_INTERVAL)


# ══════════════════════════════════════════════════════════════════════════════
# Port auto-detection
# ══════════════════════════════════════════════════════════════════════════════

def _find_port() -> Optional[str]:
    """
    Return the first serial port that looks like an Oriental Motor USB device.
    Falls back to the first available port if no vendor match is found.
    """
    candidates = list(serial.tools.list_ports.comports())
    if not candidates:
        return None
    keywords = ("oriental", "azd", "ttyacm", "ttyusb", "usbserial")
    for p in candidates:
        combined = f"{p.description} {p.manufacturer or ''}".lower()
        if any(k in combined for k in keywords):
            return p.device
    return candidates[0].device


# ══════════════════════════════════════════════════════════════════════════════
# REPL
# ══════════════════════════════════════════════════════════════════════════════

_HELP = """
  m  <pos_mm> <speed_mms>              move to absolute position (fire and return)
  m  <pos_mm> <speed_mms> w            move and wait for auto-estimated time
  m  <pos_mm> <speed_mms> <seconds>    move and wait exactly N seconds

  mt <speed_mms> <time_s>              move at speed for time  (positive = forward,
                                         negative = backward; always waits)

  h                                    home (ZHOME)
  dwell <seconds>                      pause without moving

  set accel <mm/s²>                    set hardware acceleration ramp (sent to driver)
  set decel <mm/s²>                    set hardware deceleration ramp (sent to driver)
  t <pos_mm> <speed_mms>               show travel time estimate (no move)

  s                                    status (port, heartbeat, last position)
  ?                                    show this help
  q  (or Ctrl-C)                       quit
"""


def _repl(driver: MotorDriver, accel_mms2: float, decel_mms2: float) -> None:
    print(_HELP)
    _info(f"Accel: {accel_mms2:.2f} mm/s²   Decel: {decel_mms2:.2f} mm/s²  "
          f"(use 'set accel/decel <val>' to update — sent to driver on each move)")
    print()

    while True:
        try:
            raw = input(_c("bold", "dr28t> ")).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not raw:
            continue

        parts = raw.split()
        cmd   = parts[0].lower()

        # ── quit ──
        if cmd in ("q", "quit", "exit"):
            break

        # ── help ──
        elif cmd in ("?", "help"):
            print(_HELP)

        # ── status ──
        elif cmd == "s":
            driver.status()

        # ── home ──
        elif cmd == "h":
            _info("Sending home (ZHOME)…")
            try:
                driver.home()
            except serial.SerialException as exc:
                _err(f"Serial error: {exc}")

        # ── dwell ──
        elif cmd == "dwell":
            if len(parts) != 2:
                _warn("Usage:  dwell <seconds>")
                continue
            try:
                secs = float(parts[1])
            except ValueError:
                _warn("Seconds must be a number.")
                continue
            if secs <= 0:
                _warn("Seconds must be > 0.")
                continue
            driver.wait(secs)

        # ── travel time estimate (no move) ──
        elif cmd == "t":
            if len(parts) != 3:
                _warn("Usage:  t <pos_mm> <speed_mms>")
                continue
            try:
                pos_mm  = float(parts[1])
                spd_mms = float(parts[2])
            except ValueError:
                _warn("Position and speed must be numbers.")
                continue
            distance = abs(pos_mm - (driver._last_pos_mm or 0.0))
            est_s    = _travel_time_s(distance, spd_mms, accel_mms2)
            _info(f"Travel time estimate: {est_s:.2f} s")
            _info(f"  distance={distance:.1f} mm  speed={spd_mms:.1f} mm/s  "
                  f"accel={accel_mms2:.0f} mm/s²")

        # ── set parameter ──
        elif cmd == "set":
            if len(parts) < 3:
                _warn("Usage:  set accel <mm/s²>  |  set decel <mm/s²>")
                continue
            key = parts[1].lower()
            if key == "accel":
                try:
                    accel_mms2 = float(parts[2])
                except ValueError:
                    _warn("Acceleration must be a number (mm/s²).")
                    continue
                if accel_mms2 <= 0:
                    _warn("Acceleration must be > 0.")
                    continue
                _ok(f"Acceleration updated → {accel_mms2:.2f} mm/s²  (applied on next move)")
            elif key == "decel":
                try:
                    decel_mms2 = float(parts[2])
                except ValueError:
                    _warn("Deceleration must be a number (mm/s²).")
                    continue
                if decel_mms2 <= 0:
                    _warn("Deceleration must be > 0.")
                    continue
                _ok(f"Deceleration updated → {decel_mms2:.2f} mm/s²  (applied on next move)")
            else:
                _warn(f"Unknown setting '{key}'.  Available: accel, decel")

        # ── move ──
        elif cmd == "m":
            if len(parts) < 3 or len(parts) > 4:
                _warn("Usage:  m <pos_mm> <speed_mms> [w | <wait_seconds>]")
                continue
            try:
                pos_mm  = float(parts[1])
                spd_mms = float(parts[2])
            except ValueError:
                _warn("Position and speed must be numbers.")
                continue

            if not (0.0 <= pos_mm <= MAX_POS_MM):
                _warn(f"Position must be 0 – {MAX_POS_MM} mm.")
                continue
            if not (0.1 <= spd_mms <= MAX_SPD_MMS):
                _warn(f"Speed must be 0.1 – {MAX_SPD_MMS} mm/s.")
                continue

            # Parse optional wait argument
            wait_s: Optional[float] = None
            if len(parts) == 4:
                arg = parts[3].lower()
                if arg == "w":
                    # Auto-estimate: travel time + 20 % margin + 0.3 s buffer
                    distance = abs(pos_mm - (driver._last_pos_mm or 0.0))
                    est_s    = _travel_time_s(distance, spd_mms, accel_mms2)
                    wait_s   = est_s * 1.2 + 0.3
                else:
                    try:
                        wait_s = float(arg)
                    except ValueError:
                        _warn("3rd argument must be 'w' or a number of seconds.")
                        continue
                    if wait_s < 0:
                        _warn("Wait time must be ≥ 0.")
                        continue

            _info(f"Move → {pos_mm:.2f} mm  @ {spd_mms:.2f} mm/s")
            try:
                driver.move(pos_mm, spd_mms, wait_s=wait_s,
                            accel_mms2=accel_mms2, decel_mms2=decel_mms2)
            except serial.SerialException as exc:
                _err(f"Serial error: {exc}")

        # ── move-by-time ──
        elif cmd == "mt":
            if len(parts) != 3:
                _warn("Usage:  mt <speed_mms> <time_s>")
                _warn("  positive speed = forward, negative = backward")
                continue
            try:
                spd_signed = float(parts[1])
                time_s     = float(parts[2])
            except ValueError:
                _warn("Speed and time must be numbers.")
                continue
            if abs(spd_signed) < 0.1:
                _warn("Speed magnitude must be ≥ 0.1 mm/s.")
                continue
            if time_s <= 0:
                _warn("Time must be > 0 s.")
                continue

            spd_mms   = abs(spd_signed)
            current   = driver._last_pos_mm if driver._last_pos_mm is not None else 0.0
            delta_mm  = spd_signed * time_s
            target_mm = current + delta_mm

            # Clamp to safe travel range
            target_mm = max(0.0, min(MAX_POS_MM, target_mm))
            actual_delta = abs(target_mm - current)

            if actual_delta < 0.1:
                _warn(
                    f"Target {target_mm:.2f} mm is already at or beyond the limit — "
                    "nothing to do."
                )
                continue

            if target_mm != current + delta_mm:
                _warn(
                    f"Clamped: requested {current + delta_mm:.2f} mm → {target_mm:.2f} mm "
                    f"(travel limit {MAX_POS_MM} mm)."
                )

            # Wait time = commanded duration + 20 % margin for ramp-up
            wait_s = time_s * 1.2 + 0.3

            _info(
                f"Move-by-time: {current:.2f} → {target_mm:.2f} mm  "
                f"({'+' if delta_mm >= 0 else ''}{delta_mm:.1f} mm)  "
                f"@ {spd_mms:.1f} mm/s  for {time_s:.2f} s"
            )
            try:
                driver.move(target_mm, spd_mms, wait_s=wait_s,
                            accel_mms2=accel_mms2, decel_mms2=decel_mms2)
            except serial.SerialException as exc:
                _err(f"Serial error: {exc}")

        else:
            _warn(f"Unknown command '{cmd}'.  Type ? for help.")


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(
        description="DR28T motor controller — no ROS2 required.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--port", "-p",
        default=None,
        help="Serial port (e.g. COM3 or /dev/ttyACM0).  Auto-detected if omitted.",
    )
    parser.add_argument(
        "--baud", "-b",
        type=int,
        default=19200,
        help="Baud rate (default: 19200).",
    )
    parser.add_argument(
        "--accel",
        type=float,
        default=DEFAULT_ACCEL_MMS2,
        help=f"Initial hardware acceleration ramp in mm/s² "
             f"(default: {DEFAULT_ACCEL_MMS2}).  Sent to driver on every move.",
    )
    parser.add_argument(
        "--decel",
        type=float,
        default=DEFAULT_DECEL_MMS2,
        help=f"Initial hardware deceleration ramp in mm/s² "
             f"(default: {DEFAULT_DECEL_MMS2}).  Sent to driver on every move.",
    )
    args = parser.parse_args()

    # ── Resolve port ──
    port = args.port
    if port is None:
        port = _find_port()
        if port is None:
            _err("No serial ports found.  Connect the driver and retry, or use --port.")
            sys.exit(1)
        _info(f"Auto-detected port: {port}")

    # ── Banner ──
    print()
    print(_c("bold", "  DR28T Motor Controller"))
    print(_c("gray", f"  {port}  {args.baud} baud  8E1"))
    print(_c("gray",  "  ─────────────────────────────"))

    # ── Open and run ──
    try:
        driver = MotorDriver.open(port, args.baud)
    except serial.SerialException as exc:
        _err(f"Cannot open {port}: {exc}")
        sys.exit(1)

    try:
        _repl(driver, accel_mms2=args.accel, decel_mms2=args.decel)
    finally:
        driver.close()
        print(_c("gray", "  Goodbye."))


if __name__ == "__main__":
    main()
