"""
DR28T linear motor driver — MEXE02 serial protocol over USB.

Protocol overview
─────────────────
The AZD-KEP driver communicates over a virtual serial port (USB, appears as
COMx on Windows or /dev/ttyACM0 on Linux).  Commands are binary frames
reverse-engineered from MEXE02 software captures.  The frame bytes are
hardcoded as Python constants below; only the target position and speed values
are patched at known byte offsets before each transmission.

Motion sequence (absolute move)
────────────────────────────────
1. Sync byte   (0x06)             – handshake before bulk register-write
2. Position frame (320 bytes)     – loads target position into data slot 0
3. Speed frame    (240 bytes)     – loads operating speed into data slot 0
4. Sync byte   (0x06)             – handshake before execute block
5. Execute frame 1 (40 bytes)     – triggers motion start
6. Execute frame 2 (40 bytes)     – confirms motion trigger

Homing sequence (ZHOME)
────────────────────────
1. Sync byte   (0x06)
2. Home frame 1 (40 bytes)
3. Home frame 2 (40 bytes)

Value encoding
──────────────
Position : int32 little-endian in µm (micrometres)
           Baseline 25 000 µm = 25 mm → bytes [0xa8, 0x61, 0x00, 0x00]
           Located at frame offset 130.

Speed    : int32 little-endian in µm/s
           Baseline 1 000 µm/s = 1 mm/s → bytes [0xe8, 0x03, 0x00, 0x00]
           Low word (bytes 0-1) at frame offset 142.
           High word (bytes 2-3) at frame offset 144.

Checksum update
───────────────
Each frame contains XOR checksum bytes at fixed offsets.  When a data value
changes, every affected checksum is updated in-place using the XOR delta:

    delta        = XOR(baseline_bytes) ^ XOR(new_bytes)
    new_checksum = old_checksum ^ delta

This is mathematically equivalent to recomputing the checksum from scratch
but requires only the bytes that changed.
"""

import math
import struct
import threading
import time
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Empty, String
import serial


# ── Protocol frame constants ───────────────────────────────────────────────────
#
# All frames were captured from MEXE02 v4 communicating with an AZD-KEP driver
# over COM3 (19200 baud, 8E1).  Do not edit the bytes unless re-capturing from
# MEXE02 with updated driver firmware.

_SYNC = bytes([0x06])  # ENQ byte — driver acknowledges with 0x86

# ── Position frame (320 bytes) ────────────────────────────────────────────────
# Bulk register-write that loads operating parameters for data slot 0.
# Position value (int32 LE, µm) lives at byte offset _POS_OFFSET.
# Three XOR checksum bytes are at offsets listed in _POS_CS_IDX.
_POS_FRAME = bytes([
    0xff,0x01,0x00,0x00,0xfc,0x00,0xc3,0x01,0x00,0x2a,0x02,0x00,0x00,0x00,0x20,0x2a,  # 000–015
    0x02,0x00,0x00,0x00,0x40,0x2a,0x02,0x00,0x00,0x00,0x60,0x2a,0x02,0x00,0x00,0x00,  # 016–031
    0x80,0x2a,0x02,0x00,0x00,0x00,0x00,0x68,0xff,0x00,0x00,0x00,0x00,0x00,0xa0,0x2a,  # 032–047
    0x02,0x00,0x00,0x00,0xc0,0x2a,0x02,0x00,0x00,0x00,0xe0,0x2a,0x02,0x00,0x00,0x00,  # 048–063
    0x00,0x2b,0x02,0x00,0x00,0x00,0x20,0x2b,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x77,  # 064–079
    0xff,0x00,0x00,0x00,0x40,0x2b,0x02,0x00,0x00,0x00,0x60,0x2b,0x02,0x00,0x00,0x00,  # 080–095
    0x80,0x2b,0x02,0x00,0x00,0x00,0xa0,0x2b,0x02,0x00,0x00,0x00,0xc0,0x2b,0x02,0x00,  # 096–111
    0x00,0x00,0xe0,0x2b,0x00,0x00,0x00,0xdd,0xff,0x00,0x00,0x00,0x02,0x00,0x00,0x00,  # 112–127
    0x01,0x0c,0xa8,0x61,0x00,0x00,0x21,0x0c,0x00,0x00,0x00,0x00,0x41,0x0c,0x00,0x00,  # 128–143  ← pos int32 at [130:134]
    0x00,0x00,0x61,0x0c,0x00,0x00,0x00,0x00,0x81,0x0c,0x00,0x00,0x00,0x00,0x00,0xb9,  # 144–159  ← checksum [159]
    0xff,0x00,0x00,0x00,0x00,0x00,0xa1,0x0c,0x00,0x00,0x00,0x00,0xc1,0x0c,0x00,0x00,  # 160–175
    0x00,0x00,0xe1,0x0c,0x00,0x00,0x00,0x00,0x01,0x0d,0x00,0x00,0x00,0x00,0x21,0x0d,  # 176–191
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x52,0xff,0x00,0x00,0x00,0x41,0x0d,0x00,0x00,  # 192–207
    0x00,0x00,0x61,0x0d,0x00,0x00,0x00,0x00,0x81,0x0d,0x00,0x00,0x00,0x00,0xa1,0x0d,  # 208–223
    0x00,0x00,0x00,0x00,0xc1,0x0d,0x00,0x00,0x00,0x00,0xe1,0x0d,0x00,0x00,0x00,0xdf,  # 224–239
    0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x0e,0x00,0x00,0x00,0x00,0x21,0x0e,  # 240–255
    0x00,0x00,0x00,0x00,0x41,0x0e,0x00,0x00,0x00,0x00,0x61,0x0e,0x00,0x00,0x00,0x00,  # 256–271
    0x81,0x0e,0x00,0x00,0x00,0x00,0x00,0x70,0xff,0x02,0x00,0x00,0x00,0x00,0xa1,0x0e,  # 272–287
    0x00,0x00,0x00,0x00,0xc1,0x0e,0x00,0x00,0x00,0x00,0xe1,0x0e,0x00,0x00,0x00,0x00,  # 288–303
    0x01,0x0f,0x00,0x00,0x00,0x00,0x00,0xf9,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x85,  # 304–319  ← checksums [311],[319]
])
assert len(_POS_FRAME) == 320, "Position frame length mismatch"

_POS_OFFSET   = 130             # byte index of int32 position value
_POS_BASE_UM  = 25_000          # µm encoded in the baseline frame (= 25 mm)
_POS_CS_IDX   = (159, 311, 319) # indices of the three XOR checksum bytes


# ── Speed frame (240 bytes) ───────────────────────────────────────────────────
# Bulk register-write that sets the operating speed for data slot 0.
# Speed value (int32 LE, µm/s) is split: low word at _SPD_LO_IDX,
# high word at _SPD_HI_IDX.  Three XOR checksums at _SPD_CS_IDX.
_SPD_FRAME = bytes([
    0xff,0x01,0x00,0x00,0xba,0x00,0xc3,0x01,0xc1,0x29,0x00,0x00,0x00,0x00,0xe1,0x29,  # 000–015
    0x00,0x00,0x00,0x00,0x01,0x2a,0x00,0x00,0x00,0x00,0x21,0x2a,0x00,0x00,0x00,0x00,  # 016–031
    0x41,0x2a,0x00,0x00,0x00,0x00,0x00,0xed,0xff,0x00,0x00,0x00,0x00,0x00,0x61,0x2a,  # 032–047
    0x00,0x00,0x00,0x00,0x81,0x2a,0x00,0x00,0x00,0x00,0xa1,0x2a,0x00,0x00,0x00,0x00,  # 048–063
    0xc1,0x2a,0x00,0x00,0x00,0x00,0xe1,0x2a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xb4,  # 064–079
    0xff,0x00,0x00,0x00,0x01,0x2b,0x00,0x00,0x00,0x00,0x21,0x2b,0x00,0x00,0x00,0x00,  # 080–095
    0x41,0x2b,0x00,0x00,0x00,0x00,0x61,0x2b,0x00,0x00,0x00,0x00,0x81,0x2b,0x00,0x00,  # 096–111
    0x00,0x00,0xa1,0x2b,0x00,0x00,0x00,0xdf,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  # 112–127
    0xc1,0x2b,0x00,0x00,0x00,0x00,0xe1,0x2b,0x00,0x00,0x00,0x00,0x02,0x0c,0xe8,0x03,  # 128–143  ← speed[0:2] at [142:144]
    0x00,0x00,0x22,0x0c,0xe8,0x03,0x00,0x00,0x42,0x0c,0xe8,0x03,0x00,0x00,0x00,0x5a,  # 144–159  ← speed[2:4] at [144:146], cs [159]
    0xff,0x00,0x00,0x00,0x00,0x00,0x62,0x0c,0xe8,0x03,0x00,0x00,0x82,0x0c,0xe8,0x03,  # 160–175
    0x00,0x00,0xa2,0x0c,0xe8,0x03,0x00,0x00,0xc2,0x0c,0xe8,0x03,0x00,0x00,0xe2,0x0c,  # 176–191
    0xe8,0x03,0x00,0x00,0x00,0x00,0x00,0x7a,0xff,0x02,0x00,0x00,0x02,0x0d,0xe8,0x03,  # 192–207
    0x00,0x00,0x22,0x0d,0xe8,0x03,0x00,0x00,0x42,0x0d,0xe8,0x03,0x00,0x00,0x62,0x0d,  # 208–223
    0xe8,0x03,0x00,0x00,0x00,0x58,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xa5,  # 224–239  ← checksums [229],[239]
])
assert len(_SPD_FRAME) == 240, "Speed frame length mismatch"

_SPD_LO_IDX   = 142            # byte index of int32 speed low word (bytes 0-1)
_SPD_HI_IDX   = 144            # byte index of int32 speed high word (bytes 2-3)
_SPD_BASE_UMS = 1_000          # µm/s encoded in the baseline frame (= 1 mm/s)
_SPD_CS_IDX   = (159, 229, 239)


# ── Acceleration frame (240 bytes) ────────────────────────────────────────────
# Bulk register-write that sets the operating acceleration for all 12 data slots.
# Uses the same 240-byte envelope as the speed frame but targets register 0x0C03
# (accel) instead of 0x0C02 (speed).  Value unit: 1 count = 1 nm/s² (multiply
# mm/s² by 1 000 000).  Baseline encodes 700 000 counts = 0.7 mm/s².
_ACCEL_FRAME = bytes([
    0xff,0x01,0x00,0x00,0xba,0x00,0xc3,0x01,0xc1,0x29,0x00,0x00,0x00,0x00,0xe1,0x29,  # 000-015
    0x00,0x00,0x00,0x00,0x01,0x2a,0x00,0x00,0x00,0x00,0x21,0x2a,0x00,0x00,0x00,0x00,  # 016-031
    0x41,0x2a,0x00,0x00,0x00,0x00,0x00,0xed,0xff,0x00,0x00,0x00,0x00,0x00,0x61,0x2a,  # 032-047
    0x00,0x00,0x00,0x00,0x81,0x2a,0x00,0x00,0x00,0x00,0xa1,0x2a,0x00,0x00,0x00,0x00,  # 048-063
    0xc1,0x2a,0x00,0x00,0x00,0x00,0xe1,0x2a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xb4,  # 064-079
    0xff,0x00,0x00,0x00,0x01,0x2b,0x00,0x00,0x00,0x00,0x21,0x2b,0x00,0x00,0x00,0x00,  # 080-095
    0x41,0x2b,0x00,0x00,0x00,0x00,0x61,0x2b,0x00,0x00,0x00,0x00,0x81,0x2b,0x00,0x00,  # 096-111
    0x00,0x00,0xa1,0x2b,0x00,0x00,0x00,0xdf,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  # 112-127
    0xc1,0x2b,0x00,0x00,0x00,0x00,0xe1,0x2b,0x00,0x00,0x00,0x00,0x03,0x0c,0x60,0xae,  # 128-143  <- DS0 reg 0x0C03, val lo at [142:144]
    0x0a,0x00,0x23,0x0c,0x60,0xae,0x0a,0x00,0x43,0x0c,0x60,0xae,0x0a,0x00,0x00,0x74,  # 144-159  <- val hi at [144:146], CS[159]
    0xff,0x00,0x00,0x00,0x00,0x00,0x63,0x0c,0x60,0xae,0x0a,0x00,0x83,0x0c,0x60,0xae,  # 160-175
    0x0a,0x00,0xa3,0x0c,0x60,0xae,0x0a,0x00,0xc3,0x0c,0x60,0xae,0x0a,0x00,0xe3,0x0c,  # 176-191
    0x60,0xae,0x0a,0x00,0x00,0x00,0x00,0x7a,0xff,0x02,0x00,0x00,0x03,0x0d,0x60,0xae,  # 192-207
    0x0a,0x00,0x23,0x0d,0x60,0xae,0x0a,0x00,0x43,0x0d,0x60,0xae,0x0a,0x00,0x63,0x0d,  # 208-223
    0x60,0xae,0x0a,0x00,0x00,0x58,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x8b,  # 224-239  <- CS[229], CS[239]
])
assert len(_ACCEL_FRAME) == 240, "Acceleration frame length mismatch"

_ACL_LO_IDX    = 142            # byte index of DS0 accel low word (bytes 0-1)
_ACL_HI_IDX    = 144            # byte index of DS0 accel high word (bytes 2-3)
_ACL_BASE_RAW  = 700_000        # nm/s² encoded in baseline frame (= 0.7 mm/s²)
_ACL_CS_IDX    = (159, 229, 239)


# ── Deceleration frame (240 bytes) ───────────────────────────────────────────
# Identical envelope to the acceleration frame, but all 12 DS register addresses
# are incremented by 1: 0x0C03+0x20n (accel) → 0x0C04+0x20n (decel).
# Same value unit (1 nm/s²) and same checksum scheme.
# Derived analytically from _ACCEL_FRAME; checksums verified programmatically.
_DECEL_FRAME = bytes([
    0xff,0x01,0x00,0x00,0xba,0x00,0xc3,0x01,0xc1,0x29,0x00,0x00,0x00,0x00,0xe1,0x29,  # 000-015
    0x00,0x00,0x00,0x00,0x01,0x2a,0x00,0x00,0x00,0x00,0x21,0x2a,0x00,0x00,0x00,0x00,  # 016-031
    0x41,0x2a,0x00,0x00,0x00,0x00,0x00,0xed,0xff,0x00,0x00,0x00,0x00,0x00,0x61,0x2a,  # 032-047
    0x00,0x00,0x00,0x00,0x81,0x2a,0x00,0x00,0x00,0x00,0xa1,0x2a,0x00,0x00,0x00,0x00,  # 048-063
    0xc1,0x2a,0x00,0x00,0x00,0x00,0xe1,0x2a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xb4,  # 064-079
    0xff,0x00,0x00,0x00,0x01,0x2b,0x00,0x00,0x00,0x00,0x21,0x2b,0x00,0x00,0x00,0x00,  # 080-095
    0x41,0x2b,0x00,0x00,0x00,0x00,0x61,0x2b,0x00,0x00,0x00,0x00,0x81,0x2b,0x00,0x00,  # 096-111
    0x00,0x00,0xa1,0x2b,0x00,0x00,0x00,0xdf,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  # 112-127
    0xc1,0x2b,0x00,0x00,0x00,0x00,0xe1,0x2b,0x00,0x00,0x00,0x00,0x04,0x0c,0x60,0xae,  # 128-143  <- DS0 reg 0x0C04, val lo at [142:144]
    0x0a,0x00,0x24,0x0c,0x60,0xae,0x0a,0x00,0x44,0x0c,0x60,0xae,0x0a,0x00,0x00,0x73,  # 144-159  <- val hi at [144:146], CS[159]
    0xff,0x00,0x00,0x00,0x00,0x00,0x64,0x0c,0x60,0xae,0x0a,0x00,0x84,0x0c,0x60,0xae,  # 160-175
    0x0a,0x00,0xa4,0x0c,0x60,0xae,0x0a,0x00,0xc4,0x0c,0x60,0xae,0x0a,0x00,0xe4,0x0c,  # 176-191
    0x60,0xae,0x0a,0x00,0x00,0x00,0x00,0x7a,0xff,0x02,0x00,0x00,0x04,0x0d,0x60,0xae,  # 192-207
    0x0a,0x00,0x24,0x0d,0x60,0xae,0x0a,0x00,0x44,0x0d,0x60,0xae,0x0a,0x00,0x64,0x0d,  # 208-223
    0x60,0xae,0x0a,0x00,0x00,0x58,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x8c,  # 224-239  <- CS[229], CS[239]
])
assert len(_DECEL_FRAME) == 240, "Deceleration frame length mismatch"

_DCL_LO_IDX    = 142            # byte index of DS0 decel low word (bytes 0-1)
_DCL_HI_IDX    = 144            # byte index of DS0 decel high word (bytes 2-3)
_DCL_BASE_RAW  = 700_000        # nm/s² encoded in baseline frame (= 0.7 mm/s²)
_DCL_CS_IDX    = (159, 229, 239)


# ── Execute frames (40 bytes each) ────────────────────────────────────────────
# Sent after the position/speed frames to trigger the loaded motion.
_EXEC_FRAMES: tuple[bytes, ...] = (
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

# ── Home frames (40 bytes each) ───────────────────────────────────────────────
# Sent to initiate the ZHOME (return-to-origin) sequence.
_HOME_FRAMES: tuple[bytes, ...] = (
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

# Frame repeated by the heartbeat thread to keep the driver alive.
_HEARTBEAT_FRAME: bytes = _EXEC_FRAMES[0]


# ── Frame builders ─────────────────────────────────────────────────────────────

def _xor_of(data: bytes) -> int:
    """Return the XOR of every byte in *data*."""
    result = 0
    for b in data:
        result ^= b
    return result


def _build_position_frame(position_mm: float) -> bytes:
    """
    Return a patched copy of *_POS_FRAME* encoding *position_mm*.

    Args:
        position_mm: Target absolute position in millimetres.

    Returns:
        320-byte frame with updated position value and checksums.
    """
    pos_um     = int(round(position_mm * 1_000))
    new_bytes  = struct.pack("<i", pos_um)
    base_bytes = struct.pack("<i", _POS_BASE_UM)
    xor_delta  = _xor_of(base_bytes) ^ _xor_of(new_bytes)

    frame = bytearray(_POS_FRAME)
    frame[_POS_OFFSET : _POS_OFFSET + 4] = new_bytes
    for idx in _POS_CS_IDX:
        frame[idx] ^= xor_delta
    return bytes(frame)


def _build_speed_frame(speed_mms: float) -> bytes:
    """
    Return a patched copy of *_SPD_FRAME* encoding *speed_mms*.

    The int32 speed value is split across two non-contiguous 2-byte fields
    (low word at _SPD_LO_IDX, high word at _SPD_HI_IDX).

    Args:
        speed_mms: Operating speed in mm/s (must be > 0).

    Returns:
        240-byte frame with updated speed value and checksums.
    """
    spd_ums    = int(round(speed_mms * 1_000))
    new_bytes  = struct.pack("<i", spd_ums)
    base_bytes = struct.pack("<i", _SPD_BASE_UMS)
    xor_delta  = _xor_of(base_bytes) ^ _xor_of(new_bytes)

    frame = bytearray(_SPD_FRAME)
    frame[_SPD_LO_IDX : _SPD_LO_IDX + 2] = new_bytes[0:2]
    frame[_SPD_HI_IDX : _SPD_HI_IDX + 2] = new_bytes[2:4]
    for idx in _SPD_CS_IDX:
        frame[idx] ^= xor_delta
    return bytes(frame)


def _build_accel_frame(accel_mms2: float) -> bytes:
    """
    Return a patched copy of *_ACCEL_FRAME* encoding *accel_mms2*.

    Args:
        accel_mms2: Acceleration in mm/s² (must be > 0).

    Returns:
        240-byte frame with updated acceleration value and checksums.
    """
    accel_raw  = int(round(accel_mms2 * 1_000_000))
    new_bytes  = struct.pack("<i", accel_raw)
    base_bytes = struct.pack("<i", _ACL_BASE_RAW)
    xor_delta  = _xor_of(base_bytes) ^ _xor_of(new_bytes)

    frame = bytearray(_ACCEL_FRAME)
    frame[_ACL_LO_IDX : _ACL_LO_IDX + 2] = new_bytes[0:2]
    frame[_ACL_HI_IDX : _ACL_HI_IDX + 2] = new_bytes[2:4]
    for idx in _ACL_CS_IDX:
        frame[idx] ^= xor_delta
    return bytes(frame)


def _build_decel_frame(decel_mms2: float) -> bytes:
    """
    Return a patched copy of *_DECEL_FRAME* encoding *decel_mms2*.

    Args:
        decel_mms2: Deceleration in mm/s² (must be > 0).

    Returns:
        240-byte frame with updated deceleration value and checksums.
    """
    decel_raw  = int(round(decel_mms2 * 1_000_000))
    new_bytes  = struct.pack("<i", decel_raw)
    base_bytes = struct.pack("<i", _DCL_BASE_RAW)
    xor_delta  = _xor_of(base_bytes) ^ _xor_of(new_bytes)

    frame = bytearray(_DECEL_FRAME)
    frame[_DCL_LO_IDX : _DCL_LO_IDX + 2] = new_bytes[0:2]
    frame[_DCL_HI_IDX : _DCL_HI_IDX + 2] = new_bytes[2:4]
    for idx in _DCL_CS_IDX:
        frame[idx] ^= xor_delta
    return bytes(frame)


def _move_payloads(position_mm: float, speed_mms: float, accel_mms2: float,
                   decel_mms2: float) -> List[bytes]:
    """Return the ordered frame sequence for an absolute move command."""
    return [
        _SYNC,
        _build_accel_frame(accel_mms2),
        _build_decel_frame(decel_mms2),
        _build_position_frame(position_mm),
        _build_speed_frame(speed_mms),
        _SYNC,
        *_EXEC_FRAMES,
    ]


def _home_payloads() -> List[bytes]:
    """Return the ordered frame sequence for a ZHOME homing command."""
    return [_SYNC, *_HOME_FRAMES]


def _speed_for_duration(distance_mm: float, duration_s: float, accel_mms2: float) -> float:
    """Compute the peak speed needed for a symmetric move profile."""
    if distance_mm <= 0.0:
        return 0.1
    if accel_mms2 <= 0.0:
        return distance_mm / duration_s

    min_time_s = 2.0 * math.sqrt(distance_mm / accel_mms2)
    if duration_s < min_time_s:
        raise ValueError(
            f"Requested duration {duration_s:.3f}s is shorter than the achievable minimum "
            f"{min_time_s:.3f}s for {distance_mm:.3f} mm at {accel_mms2:.3f} mm/s^2."
        )

    disc = accel_mms2 * accel_mms2 * duration_s * duration_s - 4.0 * accel_mms2 * distance_mm
    if disc <= 0.0:
        return math.sqrt(distance_mm * accel_mms2)
    return (accel_mms2 * duration_s - math.sqrt(disc)) / 2.0


# ── ROS2 Node ──────────────────────────────────────────────────────────────────

class MotorControllerNode(Node):
    """
    ROS2 node for one DR28T linear axis via MEXE02 serial protocol.

    Parameters (ROS2)
    ─────────────────
    com_port             : str   Serial device path   (default: /dev/ttyACM0)
    baudrate             : int   Baud rate            (default: 19200)
    serial_timeout       : float Read timeout (s)     (default: 0.5)
    serial_write_timeout : float Write timeout (s)    (default: 0.5)
    max_position_mm      : float Software travel limit (default: 29.0)
    frame_delay          : float Delay between frames (default: 0.1 s)
    heartbeat_interval   : float Heartbeat period (s) (default: 0.2)
    motor_id             : str   Label for log output (default: "motor")

    Subscriptions
    ─────────────
    ~/motor_cmd   [Float32MultiArray]
                    data[0]=position_mm
                    data[1]=speed_mms
                    data[2]=acceleration_mms2   (optional; default 1.0 mm/s²)
                    data[3]=deceleration_mms2   (optional; default 1.0 mm/s²)
                    data[4]=move_time_s         (optional, compute speed from time)
    ~/motor_home  [Empty]              trigger ZHOME homing sequence

    Publications
    ────────────
    ~/motor_status [String]  "idle" | "busy" | "running" | "error"
    """

    def __init__(self) -> None:
        super().__init__("motor_controller")

        self.declare_parameter("com_port",             "/dev/ttyACM0")
        self.declare_parameter("baudrate",             19200)
        self.declare_parameter("serial_timeout",       0.5)
        self.declare_parameter("serial_write_timeout", 0.5)
        self.declare_parameter("max_position_mm",      29.0)
        self.declare_parameter("max_speed_mms",        100.0)
        self.declare_parameter("default_acceleration_mms2", 1.0)
        self.declare_parameter("default_deceleration_mms2", 1.0)
        self.declare_parameter("frame_delay",          0.1)
        self.declare_parameter("heartbeat_interval",   0.2)
        self.declare_parameter("motor_id",             "motor")

        com_port             = self.get_parameter("com_port").value
        baudrate             = self.get_parameter("baudrate").value
        serial_timeout       = self.get_parameter("serial_timeout").value
        serial_write_timeout = self.get_parameter("serial_write_timeout").value
        self._max_pos        = self.get_parameter("max_position_mm").value
        self._max_spd        = self.get_parameter("max_speed_mms").value
        self._default_accel  = self.get_parameter("default_acceleration_mms2").value
        self._default_decel  = self.get_parameter("default_deceleration_mms2").value
        self._frame_delay    = self.get_parameter("frame_delay").value
        self._hb_interval    = self.get_parameter("heartbeat_interval").value
        self._motor_id       = self.get_parameter("motor_id").value
        self._last_pos_mm    = 0.0

        self._serial_lock      = threading.Lock()
        self._stop_hb          = threading.Event()
        self._heartbeat_thread: threading.Thread | None = None

        try:
            self._ser = serial.Serial(
                port=com_port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_ONE,
                timeout=serial_timeout,
                write_timeout=serial_write_timeout,
            )
            self.get_logger().info(f"[{self._motor_id}] Serial port {com_port} opened.")
        except serial.SerialException as exc:
            self.get_logger().fatal(f"[{self._motor_id}] Cannot open {com_port}: {exc}")
            raise

        self.create_subscription(Float32MultiArray, "motor_cmd",  self._on_cmd_v2,  10)
        self.create_subscription(Empty,             "motor_home", self._on_home_v2, 10)
        self._status_pub = self.create_publisher(String, "motor_status", 10)
        self._publish_status("idle")
        self.get_logger().info(
            f"[{self._motor_id}] Motor controller ready — "
            "subscribe to ~/motor_cmd or publish to ~/motor_home."
        )

    # ── Subscription callbacks ─────────────────────────────────────────────

    def _on_cmd_v2(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().error(
                "motor_cmd: expected Float32MultiArray with "
                "data[0]=position_mm and data[1]=speed_mms."
            )
            return

        pos_mm = float(msg.data[0])
        spd_mms = float(msg.data[1])
        accel_mms2 = float(msg.data[2]) if len(msg.data) >= 3 else self._default_accel
        decel_mms2 = (
            float(msg.data[3]) if len(msg.data) >= 4
            else (accel_mms2 if len(msg.data) >= 3 else self._default_decel)
        )
        duration_s = float(msg.data[4]) if len(msg.data) >= 5 else 0.0

        if not (0.0 <= pos_mm <= self._max_pos):
            self.get_logger().error(
                f"Position {pos_mm:.2f} mm is outside the allowed range "
                f"[0, {self._max_pos}] mm - command ignored."
            )
            return

        if duration_s > 0.0:
            try:
                spd_mms = _speed_for_duration(
                    distance_mm=abs(pos_mm - self._last_pos_mm),
                    duration_s=duration_s,
                    accel_mms2=accel_mms2,
                )
            except ValueError as exc:
                self.get_logger().error(f"[{self._motor_id}] {exc} Command ignored.")
                return

        if spd_mms <= 0.0 or spd_mms > self._max_spd:
            self.get_logger().error(
                f"Speed must be in the range (0, {self._max_spd}] mm/s - command ignored."
            )
            return

        self.get_logger().info(
            f"[{self._motor_id}] Move -> {pos_mm:.2f} mm @ {spd_mms:.2f} mm/s  "
            f"(accel={accel_mms2:.2f} mm/s^2  decel={decel_mms2:.2f} mm/s^2"
            + (f"  time={duration_s:.3f} s" if duration_s > 0.0 else "")
            + ")"
        )
        self._execute(_move_payloads(pos_mm, spd_mms, accel_mms2, decel_mms2))
        self._last_pos_mm = pos_mm

    def _on_home_v2(self, _msg: Empty) -> None:
        self.get_logger().info(f"[{self._motor_id}] Homing (ZHOME)...")
        self._execute(_home_payloads())
        self._last_pos_mm = 0.0

    def _on_cmd(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().error(
                "motor_cmd: expected Float32MultiArray with "
                "data[0]=position_mm and data[1]=speed_mms."
            )
            return

        pos_mm  = float(msg.data[0])
        spd_mms = float(msg.data[1])

        if not (0.0 <= pos_mm <= self._max_pos):
            self.get_logger().error(
                f"Position {pos_mm:.2f} mm is outside the allowed range "
                f"[0, {self._max_pos}] mm — command ignored."
            )
            return
        if spd_mms <= 0.0:
            self.get_logger().error("Speed must be > 0 mm/s — command ignored.")
            return

        self.get_logger().info(
            f"[{self._motor_id}] Move → {pos_mm:.2f} mm @ {spd_mms:.2f} mm/s"
        )
        self._execute(_move_payloads(pos_mm, spd_mms, self._default_accel, self._default_accel))

    def _on_home(self, _msg: Empty) -> None:
        self.get_logger().info(f"[{self._motor_id}] Homing (ZHOME)…")
        self._execute(_home_payloads())

    # ── Status publisher ───────────────────────────────────────────────────

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    # ── Serial I/O ─────────────────────────────────────────────────────────

    def _send_payloads(self, payloads: List[bytes]) -> None:
        """Write each frame in *payloads* to the serial port sequentially."""
        with self._serial_lock:
            for i, frame in enumerate(payloads):
                self.get_logger().debug(
                    f"[{self._motor_id}] TX [{i+1}/{len(payloads)}] {frame.hex(' ')}"
                )
                self._ser.write(frame)
                time.sleep(self._frame_delay)
                if self._ser.in_waiting:
                    resp = self._ser.read(self._ser.in_waiting)
                    self.get_logger().debug(
                        f"[{self._motor_id}] RX {resp.hex(' ')}"
                    )

    # ── Command execution and heartbeat ────────────────────────────────────

    def _execute(self, payloads: List[bytes]) -> None:
        """Stop any running heartbeat, send *payloads*, then restart heartbeat."""
        self._stop_heartbeat()
        self._publish_status("busy")
        self._send_payloads(payloads)
        self.get_logger().info(
            f"[{self._motor_id}] All frames sent — heartbeat running."
        )
        self._start_heartbeat()
        self._publish_status("running")

    def _start_heartbeat(self) -> None:
        self._stop_hb.clear()
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            daemon=True,
            name=f"hb-{self._motor_id}",
        )
        self._heartbeat_thread.start()

    def _stop_heartbeat(self) -> None:
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._stop_hb.set()
            self._heartbeat_thread.join(timeout=2.0)
        self._heartbeat_thread = None

    def _heartbeat_loop(self) -> None:
        """Send _HEARTBEAT_FRAME at regular intervals to keep the driver active."""
        count = 0
        while not self._stop_hb.is_set():
            with self._serial_lock:
                try:
                    self._ser.write(_HEARTBEAT_FRAME)
                except serial.SerialException as exc:
                    self.get_logger().error(
                        f"[{self._motor_id}] Heartbeat serial error: {exc}"
                    )
                    break
            count += 1
            if count % 25 == 0:
                self.get_logger().info(
                    f"[{self._motor_id}] Motor alive ({count} heartbeats).",
                    throttle_duration_sec=5.0,
                )
            self._stop_hb.wait(timeout=self._hb_interval)

    # ── Lifecycle ──────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self._stop_heartbeat()
        self._publish_status("idle")
        if self._ser and self._ser.is_open:
            self._ser.close()
        self.get_logger().info(f"[{self._motor_id}] Shutdown complete.")
        super().destroy_node()


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
