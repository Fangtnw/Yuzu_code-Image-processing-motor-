"""
replay_frames.py — send individual MEXE02 frames to the motor, part by part.

Workflow
--------
1. Run diff_dump.py on your new dump → it prints Python bytes literals.
2. Paste those literals into the "── Paste frames here ──" section below.
3. Comment/uncomment lines in the TEST SEQUENCE to isolate each part.
4. Run this script while the motor is connected on the correct COM port.

Settings
--------
  PORT  : COM port, e.g. "COM11" on Windows or "/dev/ttyACM0" on Linux
  BAUD  : 19200 (fixed for AZD-KEP MEXE02 protocol)
  DELAY : seconds to wait between frames (0.1 s is safe; reduce if too slow)
"""

import serial
import time

# ── Config ────────────────────────────────────────────────────────────────────

PORT  = "COM11"
BAUD  = 19200
DELAY = 0.1   # seconds between frames

# ── Paste frames here (from diff_dump.py output) ──────────────────────────────
#
# Replace the placeholder bytes([]) with the actual frame bytes.
# diff_dump.py prints ready-to-paste Python literals.
#
# Example (from motor_controller_node.py baseline values):
#   POSITION_FRAME contains 25 mm  (0xa8 0x61 at offset 130)
#   SPEED_FRAME    contains 1 mm/s (0xe8 0x03 at offset 142)

SYNC = bytes([0x06])

POSITION_FRAME = bytes([])   # ← paste 320-byte position frame here

SPEED_FRAME    = bytes([])   # ← paste 240-byte speed frame here

ACCEL_FRAME    = bytes([])   # ← paste acceleration frame here (once identified)

EXEC_FRAME_1   = bytes([
    0xff,0x03,0x00,0x00,0x0c,0x00,0x83,0x00,0x82,0x01,0x01,0x00,0x00,0x00,0x00,0x0d,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,
])

EXEC_FRAME_2   = bytes([
    0xff,0x03,0x00,0x00,0x0c,0x00,0x83,0x00,0x9b,0x01,0x00,0x00,0x00,0x00,0x00,0x15,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,
])


# ── Serial helpers ────────────────────────────────────────────────────────────

def send(ser: serial.Serial, label: str, frame: bytes) -> None:
    if not frame:
        print(f"  SKIP  {label}  (empty — paste frame bytes first)")
        return
    preview = frame[:12].hex(" ") + ("..." if len(frame) > 12 else "")
    print(f"  TX  {label:<16s}  {len(frame):4d} B  [{preview}]")
    ser.write(frame)
    time.sleep(DELAY)
    if ser.in_waiting:
        rx = ser.read(ser.in_waiting)
        print(f"  RX  {' '*16}  {len(rx):4d} B  [{rx[:12].hex(' ')}{'...' if len(rx) > 12 else ''}]")


# ── Test sequence — comment/uncomment to test part by part ────────────────────

def run_test(ser: serial.Serial) -> None:
    print("\n── Test sequence ────────────────────────────────────────────────")

    # --- Uncomment ACCEL_FRAME to test acceleration control ---
    # send(ser, "SYNC",         SYNC)
    # send(ser, "ACCEL_FRAME",  ACCEL_FRAME)

    # --- Standard move sequence ---
    send(ser, "SYNC",         SYNC)
    send(ser, "POSITION",     POSITION_FRAME)
    send(ser, "SPEED",        SPEED_FRAME)
    send(ser, "SYNC",         SYNC)
    send(ser, "EXEC_1",       EXEC_FRAME_1)
    send(ser, "EXEC_2",       EXEC_FRAME_2)

    print("── Done ─────────────────────────────────────────────────────────\n")


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    print(f"Opening {PORT} @ {BAUD} baud (8E1) ...")
    try:
        with serial.Serial(
            port=PORT,
            baudrate=BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5,
            write_timeout=1.0,
        ) as ser:
            print(f"Port open. Frame delay: {DELAY} s\n")
            run_test(ser)
    except serial.SerialException as exc:
        print(f"ERROR: {exc}")
        print(f"Check that {PORT} is correct and no other program (MEXE02) is using it.")


if __name__ == "__main__":
    main()
