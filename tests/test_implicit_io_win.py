"""
AZD-KEP EtherNet/IP implicit I/O test for Windows.

This script opens a real class-1 style I/O connection and exchanges cyclic
UDP data with the driver. It uses the HM-60372-7 assembly layout:

    Output assembly: instance 101, attribute 3
    Input assembly:  instance 100, attribute 3

Usage:
    python test_implicit_io_win.py --ip 192.168.2.3 --spm 100
"""

import argparse
import random
import socket
import struct
import threading
import time


EIP_TCP_PORT = 44818
EIP_UDP_PORT = 2222

CMD_REGISTER = 0x0065
CMD_SEND_RR = 0x006F

SVC_FWD_OPEN = 0x54
SVC_FWD_CLOSE = 0x4E

OT_POINT = 101   # scanner -> driver output assembly
TO_POINT = 100   # driver -> scanner input assembly

OT_SIZE = 40
TO_SIZE = 56

RPI_US = 20_000

FIO_IN_START = 0x0008
FIO_IN_ZHOME = 0x0010
FIO_IN_STOP = 0x0020
FIO_IN_FREE = 0x0040
FIO_IN_ALM_RST = 0x0080
FIO_IN_TRIG = 0x0100

FIO_OUT_MOVE = 0x0002
FIO_OUT_IN_POS = 0x0004
FIO_OUT_HOME_END = 0x0010
FIO_OUT_READY = 0x0020
FIO_OUT_DCMD_RDY = 0x0040
FIO_OUT_ALM_A = 0x0080
FIO_OUT_TRIG_R = 0x0100
FIO_OUT_SET_ERR = 0x0400
FIO_OUT_EXE_ERR = 0x0800

DCMD_INC_CMD = 2
DEFAULT_ACCEL = 1_000_000
SPM = 100.0

_OUT_FMT = struct.Struct("<HHHH iiii HHHHHH i")
_IN_FMT = struct.Struct("<HHHH iIi HH I HHHH i")
assert _OUT_FMT.size == 40, _OUT_FMT.size
assert _IN_FMT.size == 40, _IN_FMT.size


def build_output(fio_in=0, dcmd_type=0, pos=0, spd=0, acc=0, dec=0) -> bytes:
    return _OUT_FMT.pack(
        0,
        0,
        fio_in,
        dcmd_type,
        pos,
        spd,
        acc,
        dec,
        1000,   # current = 100.0%
        0,      # forwarding destination = execution memory
        0,
        0,
        0,
        0,
        0,
    )


def parse_input(data: bytes) -> dict | None:
    if len(data) < _IN_FMT.size:
        return None
    (_, _, fio_out, alarm,
     fb_pos, fb_spd, _cmd_pos,
     _torque, _cst, _info,
     _res, _rdp, _rws, _wrp, _rddata) = _IN_FMT.unpack_from(data)
    return {
        "move": bool(fio_out & FIO_OUT_MOVE),
        "in_pos": bool(fio_out & FIO_OUT_IN_POS),
        "home_end": bool(fio_out & FIO_OUT_HOME_END),
        "ready": bool(fio_out & FIO_OUT_READY),
        "dcmd_rdy": bool(fio_out & FIO_OUT_DCMD_RDY),
        "alarm": bool(fio_out & FIO_OUT_ALM_A),
        "trig_r": bool(fio_out & FIO_OUT_TRIG_R),
        "set_err": bool(fio_out & FIO_OUT_SET_ERR),
        "exe_err": bool(fio_out & FIO_OUT_EXE_ERR),
        "alarm_code": alarm,
        "fio_out": fio_out,
        "fb_pos_mm": fb_pos / SPM,
        "fb_speed_mms": fb_spd / SPM,
    }


def _eip_hdr(cmd: int, session: int, data: bytes) -> bytes:
    return struct.pack("<HHII8sI", cmd, len(data), session, 0, b"\x00" * 8, 0) + data


def _send_rr_body(cip: bytes) -> bytes:
    null_addr = struct.pack("<HH", 0x0000, 0)
    data_item = struct.pack("<HH", 0x00B2, len(cip)) + cip
    return struct.pack("<IHH", 0, 0, 2) + null_addr + data_item


class AZDImplicit:
    def __init__(self, ip: str):
        self.ip = ip
        self._session = 0
        self._tcp = None
        self._udp = None

        self._ot_conn_id = random.randint(0x10000, 0xEFFFFF)
        self._conn_serial = random.randint(1, 0xFFFE)
        self._to_conn_id = 0
        self._vendor_id = 1
        self._originator_serial = random.randint(1, 0x7FFFFFFF)

        self._enc_seq = 0
        self._conn_seq = 0
        self._stop = threading.Event()
        self._first_packet = threading.Event()

        self._out_lock = threading.Lock()
        self._out_data = build_output()

        self._in_lock = threading.Lock()
        self._in_data: dict | None = None
        self._in_event = threading.Event()

        self._used_path = None
        self._used_transport = None
        self._used_params = None

    def connect(self):
        self._tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._tcp.settimeout(5.0)
        self._tcp.connect((self.ip, EIP_TCP_PORT))
        self._session = self._register_session()
        print(f"  TCP session: 0x{self._session:08X}")

        # Prefer the exact Exclusive Owner path from the EDS file first:
        #   Connection1 Path = "20 04 2C 65 2C 64"
        # Then keep a few fallback variants for comparison.
        paths = [
            (bytes([0x20, 0x04, 0x2C, 0x65, 0x2C, 0x64]), "EDS exclusive-owner"),
            (bytes([0x2C, OT_POINT, 0x2C, TO_POINT]), "cp ot=101 to=100"),
            (bytes([0x2C, TO_POINT, 0x2C, OT_POINT]), "cp to=100 ot=101"),
            (bytes([0x2C, 0x01, 0x2C, OT_POINT, 0x2C, TO_POINT]), "cp cfg=1 ot=101 to=100"),
            (bytes([0x2C, 0x00, 0x2C, OT_POINT, 0x2C, TO_POINT]), "cp cfg=0 ot=101 to=100"),
            (bytes([0x2C, 0x66, 0x2C, OT_POINT, 0x2C, TO_POINT]), "cp cfg=102 ot=101 to=100"),
            (bytes([0x20, 0x04, 0x24, 0x01, 0x2C, OT_POINT, 0x2C, TO_POINT]), "class4 cfg=1 ot=101 to=100"),
            (bytes([0x20, 0x04, 0x24, 0x00, 0x2C, OT_POINT, 0x2C, TO_POINT]), "class4 cfg=0 ot=101 to=100"),
            (bytes([0x20, 0x04, 0x24, 0x66, 0x2C, OT_POINT, 0x2C, TO_POINT]), "class4 cfg=102 ot=101 to=100"),
        ]
        transports = [
            (0x01, "class1 cyclic client"),
            (0x21, "class1 change-of-state client"),
        ]
        # 0x4228/0x4238 matches pycomm3's standard network parameter base bits.
        param_sets = [
            ((0x4A28, 0x4A38), "ot=40 to=56 net=0x4Axx scheduled"),
            ((0x4A2C, 0x4A38), "ot=44 to=56 net=0x4Axx scheduled"),
            ((0x4228, 0x4238), "ot=40 to=56 net=0x42xx"),
            ((0x422C, 0x4238), "ot=44 to=56 net=0x42xx"),
            ((0x4028, 0x4038), "ot=40 to=56 net=0x40xx"),
            ((0x402C, 0x4038), "ot=44 to=56 net=0x40xx"),
            ((0x422C, 0x423C), "ot=44 to=60 net=0x42xx"),
            ((0x402C, 0x403C), "ot=44 to=60 net=0x40xx"),
        ]
        conn_id_modes = [
            ("ot_random_to_zero", self._ot_conn_id, 0),
            ("ot_zero_to_random", 0, self._ot_conn_id),
        ]

        for conn_path, path_desc in paths:
            for transport, transport_desc in transports:
                for (ot_params, to_params), param_desc in param_sets:
                    for mode_desc, req_ot_id, req_to_id in conn_id_modes:
                        print(
                            f"  trying FO: {path_desc}, {transport_desc}, "
                            f"{param_desc}, {mode_desc}"
                        )
                        try:
                            to_id = self._forward_open(
                                conn_path=conn_path,
                                transport=transport,
                                ot_params=ot_params,
                                to_params=to_params,
                                req_ot_id=req_ot_id,
                                req_to_id=req_to_id,
                            )
                        except RuntimeError as exc:
                            print(f"    failed: {exc}")
                            continue

                        self._to_conn_id = to_id
                        self._used_path = conn_path
                        self._used_transport = transport
                        self._used_params = (ot_params, to_params)

                        if self._start_io():
                            print(
                                f"  implicit I/O up: T->O conn=0x{self._to_conn_id:08X} "
                                f"via {path_desc}, {transport_desc}, {param_desc}, {mode_desc}"
                            )
                            return

                        self._stop_io_only()
                        try:
                            self._forward_close()
                        except Exception:
                            pass
                        self._to_conn_id = 0

        raise RuntimeError("unable to establish a working implicit I/O connection")

    def disconnect(self):
        self._stop.set()
        time.sleep(0.05)
        try:
            self._forward_close()
        except Exception:
            pass
        for sock in (self._udp, self._tcp):
            try:
                if sock:
                    sock.close()
            except Exception:
                pass

    def set_output(self, data: bytes):
        with self._out_lock:
            self._out_data = data

    def get_input(self, timeout: float = 0.1) -> dict | None:
        self._in_event.wait(timeout)
        self._in_event.clear()
        with self._in_lock:
            return self._in_data

    def _register_session(self) -> int:
        pkt = _eip_hdr(CMD_REGISTER, 0, struct.pack("<HH", 1, 0))
        self._tcp.send(pkt)
        resp = self._tcp.recv(1024)
        return struct.unpack_from("<I", resp, 4)[0]

    def _forward_open(
        self,
        conn_path: bytes,
        transport: int,
        ot_params: int,
        to_params: int,
        req_ot_id: int,
        req_to_id: int,
    ) -> int:
        # Based on a normal CIP Forward Open layout, but with class-1 transport.
        fo = bytearray()
        fo += bytes([0x0A, 0x05])  # priority / timeout ticks
        fo += struct.pack("<II", req_ot_id, req_to_id)
        fo += struct.pack("<HHI", self._conn_serial, self._vendor_id, self._originator_serial)
        fo += bytes([0x07, 0x00, 0x00, 0x00])  # timeout multiplier + reserved
        fo += struct.pack("<IH", RPI_US, ot_params)
        fo += struct.pack("<IH", RPI_US, to_params)
        fo += struct.pack("<BB", transport, len(conn_path) // 2)
        fo += conn_path

        cm_path = bytes([0x20, 0x06, 0x24, 0x01])
        cip = bytes([SVC_FWD_OPEN, len(cm_path) // 2]) + cm_path + bytes(fo)
        self._tcp.send(_eip_hdr(CMD_SEND_RR, self._session, _send_rr_body(cip)))
        resp = self._tcp.recv(4096)

        off = 40
        if len(resp) < off + 4:
            raise RuntimeError("short Forward Open response")
        svc = resp[off]
        if svc != (SVC_FWD_OPEN | 0x80):
            raise RuntimeError(f"unexpected reply service 0x{svc:02X}")
        gen_status = resp[off + 2]
        if gen_status != 0:
            ext_n = resp[off + 3]
            ext = struct.unpack_from(f"<{ext_n}H", resp, off + 4) if ext_n else ()
            raise RuntimeError(
                f"gen=0x{gen_status:02X} ext={[hex(x) for x in ext]}"
            )

        # Success: response returns O->T ID first, then T->O ID.
        res_ot_id = struct.unpack_from("<I", resp, off + 4)[0]
        res_to_id = struct.unpack_from("<I", resp, off + 8)[0]
        self._ot_conn_id = res_ot_id or req_ot_id or self._ot_conn_id
        return res_to_id or req_to_id

    def _forward_close(self):
        conn_path = self._used_path or bytes([0x2C, OT_POINT, 0x2C, TO_POINT])
        fc = bytearray()
        fc += bytes([0x0A, 0x05])
        fc += struct.pack("<HHI", self._conn_serial, self._vendor_id, self._originator_serial)
        fc += struct.pack("<BB", len(conn_path) // 2, 0)
        fc += conn_path
        cm_path = bytes([0x20, 0x06, 0x24, 0x01])
        cip = bytes([SVC_FWD_CLOSE, len(cm_path) // 2]) + cm_path + bytes(fc)
        self._tcp.send(_eip_hdr(CMD_SEND_RR, self._session, _send_rr_body(cip)))
        self._tcp.recv(1024)

    def _start_io(self) -> bool:
        self._stop.clear()
        self._first_packet.clear()

        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._udp.bind(("", EIP_UDP_PORT))
        self._udp.settimeout(0.05)

        threading.Thread(target=self._tx_loop, daemon=True, name="azd-tx").start()
        threading.Thread(target=self._rx_loop, daemon=True, name="azd-rx").start()

        if self._first_packet.wait(1.5):
            return True
        print("    no T->O UDP data after Forward Open")
        return False

    def _stop_io_only(self):
        self._stop.set()
        time.sleep(0.05)
        try:
            if self._udp:
                self._udp.close()
        except Exception:
            pass
        self._udp = None

    def _tx_loop(self):
        t_next = time.monotonic()
        while not self._stop.is_set():
            with self._out_lock:
                out = self._out_data

            self._enc_seq = (self._enc_seq + 1) & 0xFFFFFFFF
            self._conn_seq = (self._conn_seq + 1) & 0xFFFF

            # Class-1 O->T: 2-byte sequence in connected item, then 32-bit run/idle.
            payload = struct.pack("<I", 1) + out
            seq_addr = struct.pack("<HHII", 0x8002, 8, self._ot_conn_id, self._enc_seq)
            conn_data = struct.pack("<HHH", 0x00B1, len(payload) + 2, self._conn_seq) + payload
            pkt = struct.pack("<H", 2) + seq_addr + conn_data

            try:
                self._udp.sendto(pkt, (self.ip, EIP_UDP_PORT))
            except Exception as exc:
                if not self._stop.is_set():
                    print(f"\n  [TX ERR] {exc}")

            t_next += RPI_US / 1e6
            wait = t_next - time.monotonic()
            if wait > 0:
                time.sleep(wait)

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data, _addr = self._udp.recvfrom(1024)
            except socket.timeout:
                continue
            except Exception:
                break

            if len(data) < 24:
                continue
            try:
                addr_type, _addr_len = struct.unpack_from("<HH", data, 2)
                if addr_type not in (0x8002, 0x00A1):
                    continue
                addr_conn_id = struct.unpack_from("<I", data, 6)[0]
                if addr_conn_id == self._ot_conn_id:
                    continue

                item2_type = struct.unpack_from("<H", data, 14)[0]
                if item2_type != 0x00B1:
                    continue

                data_len = struct.unpack_from("<H", data, 16)[0]
                payload = data[20:20 + data_len - 2]
                st = parse_input(payload)
                if st:
                    with self._in_lock:
                        self._in_data = st
                    self._first_packet.set()
                    self._in_event.set()
            except Exception:
                pass


def wait_dcmd_rdy(conn: AZDImplicit, timeout=5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        st = conn.get_input(0.05)
        if st is None:
            continue
        if st["alarm"]:
            print(f"  [ALARM] code=0x{st['alarm_code']:04x}")
            return False
        if st["dcmd_rdy"]:
            return True
    print("  [TIMEOUT] DCMD-RDY not received")
    return False


def cmd_move(conn: AZDImplicit, pos_mm: float, spd_mms: float) -> bool:
    pos_steps = int(round(pos_mm * SPM))
    speed_hz = max(1, int(round(spd_mms * SPM)))
    print(f"  move -> {pos_mm:.2f} mm  {spd_mms:.2f} mm/s  ({pos_steps} steps  {speed_hz} Hz)")

    if not wait_dcmd_rdy(conn):
        return False

    conn.set_output(build_output(
        dcmd_type=DCMD_INC_CMD,
        pos=pos_steps,
        spd=speed_hz,
        acc=DEFAULT_ACCEL,
        dec=DEFAULT_ACCEL,
    ))
    time.sleep(0.06)

    conn.set_output(build_output(
        fio_in=FIO_IN_TRIG,
        dcmd_type=DCMD_INC_CMD,
        pos=pos_steps,
        spd=speed_hz,
        acc=DEFAULT_ACCEL,
        dec=DEFAULT_ACCEL,
    ))

    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        st = conn.get_input(0.05)
        if st and st["trig_r"]:
            print(f"  TRIG_R received ({(time.monotonic() - t0) * 1000:.0f} ms)")
            break
    else:
        print("  [WARN] TRIG_R not received")
        st = conn.get_input(0.05)
        if st:
            print(
                f"  [DBG] fio_out=0x{st['fio_out']:04x} ready={st['ready']} "
                f"dcmd_rdy={st['dcmd_rdy']} move={st['move']} "
                f"in_pos={st['in_pos']} set_err={st['set_err']} "
                f"exe_err={st['exe_err']} alarm={st['alarm']}"
            )

    conn.set_output(build_output(
        dcmd_type=DCMD_INC_CMD,
        pos=pos_steps,
        spd=speed_hz,
        acc=DEFAULT_ACCEL,
        dec=DEFAULT_ACCEL,
    ))
    return True


def cmd_home(conn: AZDImplicit):
    print("  homing (ZHOME)...")
    conn.set_output(build_output(fio_in=FIO_IN_ZHOME))
    time.sleep(0.1)
    conn.set_output(build_output())


def cmd_stop(conn: AZDImplicit):
    print("  stop")
    conn.set_output(build_output(fio_in=FIO_IN_STOP))
    time.sleep(0.1)
    conn.set_output(build_output())


def cmd_alarm_reset(conn: AZDImplicit):
    print("  alarm reset")
    conn.set_output(build_output(fio_in=FIO_IN_ALM_RST))
    time.sleep(0.2)
    conn.set_output(build_output())


def cmd_status(conn: AZDImplicit):
    st = conn.get_input(0.1)
    if st is None:
        print("  no T->O data received yet")
        return
    print(
        f"  pos={st['fb_pos_mm']:.2f} mm  spd={st['fb_speed_mms']:.2f} mm/s  "
        f"fio_out=0x{st['fio_out']:04x}"
    )
    flags = [
        k for k in (
            "move", "in_pos", "home_end", "ready", "dcmd_rdy",
            "alarm", "trig_r", "set_err", "exe_err"
        ) if st[k]
    ]
    print(f"  flags: {flags or ['(none)']}")
    if st["alarm"]:
        print(f"  alarm_code: 0x{st['alarm_code']:04x}")


def cmd_wait(conn: AZDImplicit, timeout=30.0):
    print("  waiting for motion to complete...", end="", flush=True)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        st = conn.get_input(0.1)
        if st is None:
            continue
        if st["alarm"] or st["set_err"] or st["exe_err"]:
            print(
                f"\n  [ERR] alarm={st['alarm']} set_err={st['set_err']} "
                f"exe_err={st['exe_err']}"
            )
            conn.set_output(build_output())
            return
        if st["in_pos"]:
            conn.set_output(build_output())
            print(f"  done  pos={st['fb_pos_mm']:.2f} mm")
            return
        print(".", end="", flush=True)
    print("\n  [TIMEOUT] motion did not complete")
    conn.set_output(build_output())


def main():
    parser = argparse.ArgumentParser(description="AZD-KEP EtherNet/IP implicit I/O test")
    parser.add_argument("--ip", default="192.168.2.3", help="Driver IP")
    parser.add_argument("--spm", default=100.0, type=float, help="Steps per mm")
    args = parser.parse_args()

    global SPM
    SPM = args.spm

    print(f"Connecting to {args.ip} via EtherNet/IP implicit I/O  (steps/mm={SPM})")
    conn = AZDImplicit(args.ip)
    try:
        conn.connect()
    except Exception as exc:
        print(f"Connection failed: {exc}")
        return

    st = conn.get_input(2.0)
    if st:
        print(f"Connected. fio_out=0x{st['fio_out']:04x}")
    else:
        print("Connected, but no T->O data yet.")

    print("\nCommands: home | move <mm> <mm/s> | stop | status | rst | wait | q\n")

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
                cmd_home(conn)
            elif cmd == "move":
                if len(parts) < 3:
                    print("  usage: move <pos_mm> <speed_mms>")
                else:
                    cmd_move(conn, float(parts[1]), float(parts[2]))
            elif cmd == "stop":
                cmd_stop(conn)
            elif cmd == "status":
                cmd_status(conn)
            elif cmd == "rst":
                cmd_alarm_reset(conn)
            elif cmd == "wait":
                cmd_wait(conn)
            else:
                print("  unknown command")
    finally:
        conn.set_output(build_output())
        time.sleep(0.05)
        conn.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
