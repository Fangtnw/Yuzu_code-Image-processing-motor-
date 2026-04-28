"""
Standalone Windows test script for DR28T via AZD-KEP EtherNet/IP.
No ROS2 required — uses pycomm3 directly.

Usage:
    pip install pycomm3
    python test_ethernet_ip_win.py [--ip 192.168.1.2] [--spm 100.0]

Key commands at the interactive prompt:
    home             — ZHOME (return to home)
    move <mm> <mms>  — absolute move, e.g.  move 10 5
    stop             — deceleration stop
    status           — read and print input assembly
    rst              — reset alarm
    q / quit         — exit
"""

import argparse
import struct
import time

try:
    from pycomm3 import CIPDriver
except ImportError:
    raise SystemExit("pycomm3 not installed.  Run:  pip install pycomm3")

# ── Assembly instance numbers ──────────────────────────────────────────────────
ASSEMBLY_OUTPUT = 101   # scanner → driver  40 bytes
ASSEMBLY_INPUT  = 100   # driver → scanner  56 bytes

# ── Fixed I/O (IN) bit masks — output bytes 4-5 ───────────────────────────────
FIO_IN_START   = 0x0008
FIO_IN_ZHOME   = 0x0010
FIO_IN_STOP    = 0x0020
FIO_IN_FREE    = 0x0040
FIO_IN_ALM_RST = 0x0080
FIO_IN_TRIG    = 0x0100

# ── Fixed I/O (OUT) bit masks — input bytes 4-5 ───────────────────────────────
FIO_OUT_MOVE     = 0x0002
FIO_OUT_IN_POS   = 0x0004
FIO_OUT_HOME_END = 0x0010
FIO_OUT_READY    = 0x0020
FIO_OUT_DCMD_RDY = 0x0040
FIO_OUT_ALM_A    = 0x0080
FIO_OUT_TRIG_R   = 0x0100
FIO_OUT_SET_ERR  = 0x0400
FIO_OUT_EXE_ERR  = 0x0800

DCMD_ABS = 1
_DEFAULT_ACCEL_RAW = 1_000_000  # 1 kHz/s

# Output 38 bytes: all fields, write-data as uint16
_OUT_FMT = struct.Struct("<HHHH iiii HHHHHH H")
# Input 40 bytes: R-OUT(H) op_num_r(H) FIO_OUT(H) alarm(H) fb_pos(i) fb_spd(I) cmd_pos(i) torque(H) cst(H) info(I) reserved(H) rd_param_r(H) rw_status(H) wr_param_r(H) rd_data(i)
_IN_FMT  = struct.Struct("<HHHH iIi HH I HHHH i")
assert _OUT_FMT.size == 38, _OUT_FMT.size
assert _IN_FMT.size  == 40, _IN_FMT.size

_SVC_GET   = 0x0E
_SVC_SET   = 0x10
_CLS_ASM   = 0x04
_ATTR_DATA = 3


# ── Packet builders / parsers ──────────────────────────────────────────────────

def build_output(fixed_io_in=0, dcmd_type=0, pos_steps=0, speed_hz=0,
                 accel_raw=0, decel_raw=0) -> bytes:
    return _OUT_FMT.pack(
        0, 0, fixed_io_in, dcmd_type,
        pos_steps, speed_hz, accel_raw, decel_raw,
        1000, 0,        # current 100%, fwd_dest=exec mem
        0, 0, 0, 0, 0,  # reserved, read_param_id, write_req, write_param_id, write_data
    )


def parse_input(data: bytes) -> dict:
    (_, _, fio_out, alarm_code,
     fb_pos, fb_speed, _cmd_pos,
     _torque, _cst, _info,
     _res, _rdp, _rws, _wrp,
     _rddata) = _IN_FMT.unpack_from(data)
    return {
        "move":      bool(fio_out & FIO_OUT_MOVE),
        "in_pos":    bool(fio_out & FIO_OUT_IN_POS),
        "home_end":  bool(fio_out & FIO_OUT_HOME_END),
        "ready":     bool(fio_out & FIO_OUT_READY),
        "dcmd_rdy":  bool(fio_out & FIO_OUT_DCMD_RDY),
        "alarm":     bool(fio_out & FIO_OUT_ALM_A),
        "trig_r":    bool(fio_out & FIO_OUT_TRIG_R),
        "set_err":   bool(fio_out & FIO_OUT_SET_ERR),
        "exe_err":   bool(fio_out & FIO_OUT_EXE_ERR),
        "alarm_code":   alarm_code,
        "fio_out":      fio_out,
        "fb_pos_mm":    fb_pos / SPM,
        "fb_speed_mms": fb_speed / SPM,
    }


# ── Driver helpers ─────────────────────────────────────────────────────────────

def write_output(drv: CIPDriver, data: bytes) -> bool:
    resp = drv.generic_message(
        service=_SVC_SET, class_code=_CLS_ASM,
        instance=ASSEMBLY_OUTPUT, attribute=_ATTR_DATA,
        request_data=data, connected=False,
    )
    if resp.error:
        print(f"  [ERR] write: {resp.error}")
        return False
    return True


def read_input(drv: CIPDriver) -> dict | None:
    resp = drv.generic_message(
        service=_SVC_GET, class_code=_CLS_ASM,
        instance=ASSEMBLY_INPUT, attribute=_ATTR_DATA,
        connected=False,
    )
    if resp.error or resp.value is None:
        print(f"  [ERR] read: {resp.error}")
        return None
    return parse_input(bytes(resp.value))


def wait_dcmd_rdy(drv: CIPDriver, timeout=5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        st = read_input(drv)
        if st is None:
            time.sleep(0.05)
            continue
        if st["alarm"]:
            print(f"  [ALARM] code=0x{st['alarm_code']:04x}")
            return False
        if st["dcmd_rdy"]:
            return True
        time.sleep(0.02)
    print("  [TIMEOUT] DCMD-RDY not received")
    return False


def cmd_move(drv: CIPDriver, pos_mm: float, spd_mms: float) -> bool:
    pos_steps = int(round(pos_mm  * SPM))
    speed_hz  = max(1, int(round(spd_mms * SPM)))
    print(f"  move → {pos_mm:.2f} mm  {spd_mms:.2f} mm/s  ({pos_steps} steps  {speed_hz} Hz)")

    if not wait_dcmd_rdy(drv):
        return False

    # Write data with TRIG=0
    print("  [DBG] writing data (TRIG=0)…")
    if not write_output(drv, build_output(
            dcmd_type=DCMD_ABS, pos_steps=pos_steps, speed_hz=speed_hz,
            accel_raw=_DEFAULT_ACCEL_RAW, decel_raw=_DEFAULT_ACCEL_RAW)):
        return False

    time.sleep(0.05)   # longer settle

    # Rising edge TRIG=1
    print("  [DBG] setting TRIG=1…")
    if not write_output(drv, build_output(
            fixed_io_in=FIO_IN_TRIG,
            dcmd_type=DCMD_ABS, pos_steps=pos_steps, speed_hz=speed_hz,
            accel_raw=_DEFAULT_ACCEL_RAW, decel_raw=_DEFAULT_ACCEL_RAW)):
        return False

    # Poll for TRIG_R acknowledgment (up to 500ms)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.5:
        st = read_input(drv)
        if st and st["trig_r"]:
            print(f"  [DBG] TRIG_R acknowledged after {(time.monotonic()-t0)*1000:.0f}ms")
            break
        time.sleep(0.01)
    else:
        print("  [DBG] TRIG_R never set — driver did not acknowledge trigger!")

    # Status snapshot after trigger
    st = read_input(drv)
    if st:
        print(f"  [DBG] post-trig: fio_out=0x{st['fio_out']:04x}  "
              f"raw_steps={int(st['fb_pos_mm']*SPM)}  "
              f"move={st['move']} set_err={st['set_err']} exe_err={st['exe_err']}")

    # Clear TRIG=0
    print("  [DBG] clearing TRIG=0")
    return write_output(drv, build_output(
        fixed_io_in=0,
        dcmd_type=DCMD_ABS, pos_steps=pos_steps, speed_hz=speed_hz,
        accel_raw=_DEFAULT_ACCEL_RAW, decel_raw=_DEFAULT_ACCEL_RAW))


def cmd_home(drv: CIPDriver) -> bool:
    print("  homing (ZHOME)…")
    return write_output(drv, build_output(fixed_io_in=FIO_IN_ZHOME))


def cmd_stop(drv: CIPDriver):
    print("  stop")
    write_output(drv, build_output(fixed_io_in=FIO_IN_STOP))


def cmd_alarm_reset(drv: CIPDriver):
    print("  alarm reset")
    write_output(drv, build_output(fixed_io_in=FIO_IN_ALM_RST))
    time.sleep(0.1)
    write_output(drv, build_output())


def cmd_status(drv: CIPDriver):
    st = read_input(drv)
    if st is None:
        return
    print(f"  pos={st['fb_pos_mm']:.2f} mm  spd={st['fb_speed_mms']:.2f} mm/s")
    flags = [k for k in ("move","in_pos","home_end","ready","dcmd_rdy","alarm","set_err","exe_err") if st[k]]
    print(f"  flags: {flags or ['(none)']}")
    if st["alarm"]:
        print(f"  alarm_code: 0x{st['alarm_code']:04x}")


def wait_for_idle(drv: CIPDriver, timeout=30.0):
    """Block until IN-POS or HOME-END, then clear control bits."""
    print("  waiting for motion to complete…", end="", flush=True)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        st = read_input(drv)
        if st is None:
            time.sleep(0.1)
            continue
        if st["alarm"] or st["set_err"] or st["exe_err"]:
            print(f"\n  [ERR] flags: alarm={st['alarm']} set_err={st['set_err']} exe_err={st['exe_err']}")
            return
        if st["in_pos"] or st["home_end"]:
            write_output(drv, build_output())   # clear all control bits
            print(f"  done  pos={st['fb_pos_mm']:.2f} mm")
            return
        print(".", end="", flush=True)
        time.sleep(0.1)
    print("\n  [TIMEOUT] motion did not complete")


# ── Main REPL ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip",  default="192.168.1.2", help="AZD-KEP IP address")
    parser.add_argument("--spm", default=100.0, type=float, help="Steps per mm")
    args = parser.parse_args()

    global SPM
    SPM = args.spm

    print(f"Connecting to AZD-KEP at {args.ip}  (steps/mm={SPM})…")
    drv = CIPDriver(args.ip)
    drv.open()
    print("Connected.\n")
    print("Commands: home | move <mm> <mm/s> | stop | status | rst | wait | q")
    print()

    try:
        while True:
            try:
                line = input(">>> ").strip()
            except EOFError:
                break

            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            if cmd in ("q", "quit", "exit"):
                break
            elif cmd == "home":
                cmd_home(drv)
            elif cmd == "move":
                if len(parts) < 3:
                    print("  usage: move <pos_mm> <speed_mms>")
                else:
                    cmd_move(drv, float(parts[1]), float(parts[2]))
            elif cmd == "stop":
                cmd_stop(drv)
            elif cmd == "status":
                cmd_status(drv)
            elif cmd == "rst":
                cmd_alarm_reset(drv)
            elif cmd == "wait":
                wait_for_idle(drv)
            else:
                print("  unknown command")

    finally:
        write_output(drv, build_output())  # clear all bits on exit
        drv.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
