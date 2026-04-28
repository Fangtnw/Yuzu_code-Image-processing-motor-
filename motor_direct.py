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
    m <pos_mm> <speed_mms>   absolute move  (pos: 0–29 mm, speed: 0.1–100 mm/s)
    h                        home (ZHOME)
    s                        show connection / heartbeat status
    q  (or Ctrl-C)           quit

Dependencies
────────────
    pip install pyserial
"""

import argparse
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
    codes = {"red": "31", "green": "32", "yellow": "33", "cyan": "36", "gray": "90", "bold": "1"}
    return f"\033[{codes.get(code, '0')}m{text}\033[0m"


def _ok(msg: str)   -> None: print(_c("green",  f"  [✓] {msg}"))
def _info(msg: str) -> None: print(_c("cyan",   f"  [i] {msg}"))
def _warn(msg: str) -> None: print(_c("yellow", f"  [!] {msg}"))
def _err(msg: str)  -> None: print(_c("red",    f"  [✗] {msg}"))


# ══════════════════════════════════════════════════════════════════════════════
# Protocol constants — identical to motor_controller_node.py
# (MEXE02 v4 captures, AZD-KEP, COM3, 19200 8E1)
# ══════════════════════════════════════════════════════════════════════════════

_SYNC = bytes([0x06])  # ENQ — driver acknowledges with 0x86

# ── Position frame (320 bytes) ─────────────────────────────────────────────────
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
_SPD_FRAME = bytes([
    0xff,0x01,0x00,0x00,0xba,0x00,0xc3,0x01,0xc1,0x29,0x00,0x00,0x00,0x00,0xe1,0x29,  # 000–015
    0x00,0x00,0x00,0x00,0x01,0x2a,0x00,0x00,0x00,0x00,0x21,0x2a,0x00,0x00,0x00,0x00,  # 016–031
    0x41,0x2a,0x00,0x00,0x00,0x00,0x00,0xed,0xff,0x00,0x00,0x00,0x00,0x00,0x61,0x2a,  # 032–047
    0x00,0x00,0x00,0x00,0x81,0x2a,0x00,0x00,0x00,0x00,0xa1,0x2a,0x00,0x00,0x00,0x00,  # 048–063
    0xc1,0x2a,0x00,0x00,0x00,0x00,0xe1,0x2a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xb4,  # 064–079
    0xff,0x00,0x00,0x00,0x01,0x2b,0x00,0x00,0x00,0x00,0x21,0x2b,0x00,0x00,0x00,0x00,  # 080–095
    0x41,0x2b,0x00,0x00,0x00,0x00,0x61,0x2b,0x00,0x00,0x00,0x00,0x81,0x2b,0x00,0x00,  # 096–111
    0x00,0x00,0xa1,0x2b,0x00,0x00,0x00,0xdf,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  # 112–127
    0xc1,0x2b,0x00,0x00,0x00,0x00,0xe1,0x2b,0x00,0x00,0x00,0x00,0x02,0x0c,0xe8,0x03,  # 128–143  ← spd[0:2] [142:144]
    0x00,0x00,0x22,0x0c,0xe8,0x03,0x00,0x00,0x42,0x0c,0xe8,0x03,0x00,0x00,0x00,0x5a,  # 144–159  ← spd[2:4] [144:146], cs [159]
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


def _move_payloads(pos_mm: float, spd_mms: float) -> List[bytes]:
    return [_SYNC, _build_position_frame(pos_mm), _build_speed_frame(spd_mms),
            _SYNC, *_EXEC_FRAMES]


def _home_payloads() -> List[bytes]:
    return [_SYNC, *_HOME_FRAMES]


# ══════════════════════════════════════════════════════════════════════════════
# Motor driver (no ROS2)
# ══════════════════════════════════════════════════════════════════════════════

MAX_POS_MM  = 29.0
MAX_SPD_MMS = 100.0
FRAME_DELAY = 0.10   # seconds between consecutive frame writes
HB_INTERVAL = 0.20   # heartbeat period


class MotorDriver:
    """
    Manages the serial port and heartbeat for one DR28T axis.

    Usage:
        driver = MotorDriver.open("/dev/ttyACM0")
        driver.move(15.0, 10.0)
        driver.home()
        driver.close()
    """

    def __init__(self, ser: serial.Serial) -> None:
        self._ser            = ser
        self._lock           = threading.Lock()
        self._stop_hb        = threading.Event()
        self._hb_thread: Optional[threading.Thread] = None
        self._hb_count       = 0
        self._last_cmd: Optional[str] = None

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

    def move(self, pos_mm: float, spd_mms: float) -> None:
        """Send an absolute move command and restart the heartbeat."""
        self._last_cmd = f"move pos={pos_mm:.2f} mm  speed={spd_mms:.2f} mm/s"
        self._send(_move_payloads(pos_mm, spd_mms))

    def home(self) -> None:
        """Trigger the ZHOME homing sequence."""
        self._last_cmd = "home (ZHOME)"
        self._send(_home_payloads())

    def status(self) -> None:
        """Print current connection and heartbeat status."""
        port   = self._ser.port
        alive  = self._hb_thread is not None and self._hb_thread.is_alive()
        hb_str = _c("green", f"running  ({self._hb_count} beats)") if alive else _c("gray", "stopped")
        _info(f"Port      : {port}")
        _info(f"Heartbeat : {hb_str}")
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

    def _start_heartbeat(self) -> None:
        self._stop_hb.clear()
        self._hb_count  = 0
        self._hb_thread = threading.Thread(
            target=self._hb_loop, daemon=True, name="dr28t-heartbeat"
        )
        self._hb_thread.start()
        _info("Heartbeat running  (motor will stay active)")

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

    # Prefer ports whose description / manufacturer contains known keywords
    keywords = ("oriental", "azd", "ttyacm", "ttyusb", "usbserial")
    for p in candidates:
        combined = f"{p.description} {p.manufacturer or ''}".lower()
        if any(k in combined for k in keywords):
            return p.device

    # Fall back: return the first available port and let the user decide
    return candidates[0].device


# ══════════════════════════════════════════════════════════════════════════════
# REPL
# ══════════════════════════════════════════════════════════════════════════════

_HELP = """
  m <pos_mm> <speed_mms>   absolute move  (pos 0–29 mm, speed 0.1–100 mm/s)
  h                        home (ZHOME)
  s                        status
  ?                        show this help
  q                        quit
"""


def _repl(driver: MotorDriver) -> None:
    print(_HELP)
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

        if cmd in ("q", "quit", "exit"):
            break

        elif cmd in ("?", "help"):
            print(_HELP)

        elif cmd == "s":
            driver.status()

        elif cmd == "h":
            _info("Sending home (ZHOME)…")
            try:
                driver.home()
            except serial.SerialException as exc:
                _err(f"Serial error: {exc}")

        elif cmd == "m":
            if len(parts) != 3:
                _warn("Usage:  m <pos_mm> <speed_mms>")
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

            _info(f"Move → {pos_mm:.2f} mm  @ {spd_mms:.2f} mm/s")
            try:
                driver.move(pos_mm, spd_mms)
            except serial.SerialException as exc:
                _err(f"Serial error: {exc}")

        else:
            _warn(f"Unknown command '{cmd}'.  Type ? for help.")


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(
        description="DR28T motor controller — no ROS2 required."
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
    args = parser.parse_args()

    # ── Resolve port ──
    port = args.port
    if port is None:
        port = _find_port()
        if port is None:
            _err("No serial ports found.  Connect the driver and retry, or use --port.")
            sys.exit(1)
        _info(f"Auto-detected port: {port}")

    # ── Print banner ──
    print()
    print(_c("bold", "  DR28T Motor Controller"))
    print(_c("gray", f"  {port}  {args.baud} baud  8E1"))
    print(_c("gray",  "  ─────────────────────────"))

    # ── Open driver and start REPL ──
    try:
        driver = MotorDriver.open(port, args.baud)
    except serial.SerialException as exc:
        _err(f"Cannot open {port}: {exc}")
        sys.exit(1)

    try:
        _repl(driver)
    finally:
        driver.close()
        print(_c("gray", "  Goodbye."))


if __name__ == "__main__":
    main()
