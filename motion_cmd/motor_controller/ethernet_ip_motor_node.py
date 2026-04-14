"""
ROS2 node — DR28T linear motor via AZD-KEP EtherNet/IP (explicit CIP messaging).
Based on Oriental Motor AZ Series EtherNet/IP User Manual (HM-60372E).

Requires:
    pip install pycomm3

Hardware setup:
    1. Set the AZD-KEP IP address via the rotary switches on the driver face:
       IP ADDR ×16 and ×1 dials → IP = 192.168.1.(16*x16 + x1)
       e.g. ×16=0, ×1=2  →  192.168.1.2
    2. Connect LAN cable: PC ↔ driver (direct or via switch).
    3. Set PC NIC to same subnet: e.g. 192.168.1.100 / 255.255.255.0

─────────────────────────────────────────────────────────────────────────────
EtherNet/IP assembly instances (AZD-KEP, per manual §4):
    Output  Instance 101 (0x65)  scanner → driver  40 bytes
    Input   Instance 100 (0x64)  driver → scanner  56 bytes

Output assembly byte layout (O→T, 40 bytes, little-endian):
    [0:2]   Remote I/O (R-IN)                  — set to 0 (unused)
    [2:4]   Operation data number selection     — 0 (not used for direct data)
    [4:6]   Fixed I/O (IN)  control bits:
              bit 3  (0x0008)  START      — execute stored-data operation
              bit 4  (0x0010)  ZHOME      — high-speed return-to-home
              bit 5  (0x0020)  STOP       — deceleration stop
              bit 6  (0x0040)  FREE       — de-energise motor
              bit 7  (0x0080)  ALM-RST    — reset alarm
              bit 8  (0x0100)  TRIG       — trigger direct data op (rising edge)
              bit 9  (0x0200)  TRIG-MODE  — 0=ON edge (default), 1=ON level
    [6:8]   Direct data operation type         — 1 = absolute positioning
    [8:12]  Direct data operation position     — int32, steps (signed)
    [12:16] Direct data operation speed        — int32, Hz (steps/s)
    [16:20] Direct data operation starting/changing rate  — int32, 1 = 0.001 kHz/s
    [20:24] Direct data operation stopping deceleration   — int32, 1 = 0.001 kHz/s
    [24:26] Direct data operation operating current       — uint16, 1=0.1% (1000=100%)
    [26:28] Direct data operation forwarding destination  — uint16, 0=execution memory
    [28:30] Reserved
    [30:32] Read parameter ID
    [32:34] Write request                      — bit 0 = WR-REQ
    [34:36] Write parameter ID
    [36:40] Write data                         — int32

Input assembly byte layout (T→O, 56 bytes, little-endian):
    [0:2]   Remote I/O (R-OUT)
    [2:4]   Operation data number selection_R
    [4:6]   Fixed I/O (OUT)  status bits:
              bit 1  (0x0002)  MOVE       — motor is operating
              bit 2  (0x0004)  IN-POS     — positioning complete
              bit 4  (0x0010)  HOME-END   — return-to-home complete
              bit 5  (0x0020)  READY      — driver ready to operate
              bit 6  (0x0040)  DCMD-RDY   — ready for direct data operation
              bit 7  (0x0080)  ALM-A      — alarm active (normally open)
              bit 8  (0x0100)  TRIG_R     — TRIG response
              bit 10 (0x0400)  SET-ERR    — direct data settings error
              bit 11 (0x0800)  EXE-ERR    — direct data execution failed
    [6:8]   Present alarm code             — uint16
    [8:12]  Feedback position              — int32, steps
    [12:16] Feedback speed                 — uint32, Hz
    [16:20] Command position               — int32, steps
    [20:22] Torque monitor
    [22:24] CST operating current
    [24:28] Information
    [28:30] Reserved
    [30:32] Read parameter ID_R
    [32:34] Read/write status
    [34:36] Write parameter ID_R
    [36:40] Read data
    [40:56] Assignable monitors 0-3

─────────────────────────────────────────────────────────────────────────────
ROS2 topic API (drop-in replacement for motor_controller_node.py):
    Subscribe  ~/motor_cmd    std_msgs/Float32MultiArray  [pos_mm, spd_mms]
    Subscribe  ~/motor_home   std_msgs/Empty
    Publish    ~/motor_status std_msgs/String   idle|busy|running|error

ROS2 parameters:
    ip_address          str    "192.168.1.2"
    motor_id            str    "motor"
    steps_per_mm        float  100.0   Steps per mm — MUST match your hardware!
                                        (DR28T: check encoder resolution × screw pitch)
    max_position_mm     float  29.0    Soft upper limit (mm)
    min_speed_mms       float  0.1     Minimum commanded speed (mm/s)
    max_speed_mms       float  100.0   Maximum commanded speed (mm/s)
    acceleration_mms2   float  0.0     Move accel (mm/s²). 0 = driver default
    deceleration_mms2   float  0.0     Move decel (mm/s²). 0 = driver default
    poll_interval_s     float  0.05    Status polling period (s)
    dcmd_rdy_timeout_s  float  3.0     Timeout waiting for DCMD-RDY before move
"""

import struct
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Empty, String

try:
    from pycomm3 import CIPDriver
    HAS_PYCOMM3 = True
except ImportError:
    HAS_PYCOMM3 = False


# ── Assembly instance numbers ──────────────────────────────────────────────────
ASSEMBLY_OUTPUT = 101   # 0x65  scanner → driver  40 bytes
ASSEMBLY_INPUT  = 100   # 0x64  driver → scanner  56 bytes

# ── Fixed I/O (IN) bit masks — Output assembly bytes 4,5 ──────────────────────
FIO_IN_START    = 0x0008  # bit 3  stored-data operation
FIO_IN_ZHOME    = 0x0010  # bit 4  high-speed return-to-home
FIO_IN_STOP     = 0x0020  # bit 5  deceleration stop
FIO_IN_FREE     = 0x0040  # bit 6  de-energise
FIO_IN_ALM_RST  = 0x0080  # bit 7  reset alarm
FIO_IN_TRIG     = 0x0100  # bit 8  trigger direct data operation (rising edge)
# bit 9 (TRIG-MODE) = 0  →  ON-edge mode (default, never set)

# ── Fixed I/O (OUT) bit masks — Input assembly bytes 4,5 ──────────────────────
FIO_OUT_MOVE     = 0x0002  # bit 1  motor is operating
FIO_OUT_IN_POS   = 0x0004  # bit 2  positioning complete
FIO_OUT_HOME_END = 0x0010  # bit 4  return-to-home complete
FIO_OUT_READY    = 0x0020  # bit 5  driver ready to operate
FIO_OUT_DCMD_RDY = 0x0040  # bit 6  ready for direct data operation
FIO_OUT_ALM_A    = 0x0080  # bit 7  alarm active (normally open)
FIO_OUT_TRIG_R   = 0x0100  # bit 8  TRIG acknowledged by driver
FIO_OUT_SET_ERR  = 0x0400  # bit 10 direct data settings error
FIO_OUT_EXE_ERR  = 0x0800  # bit 11 direct data execution error

# ── Direct data operation types ────────────────────────────────────────────────
DCMD_ABS = 1   # Absolute positioning

# ── Default acceleration/deceleration (1,000 kHz/s, unit = 0.001 kHz/s) ───────
_DEFAULT_ACCEL_RAW = 1_000_000  # = 1,000 kHz/s (driver initial value)

# ── Assembly struct formats (little-endian) ───────────────────────────────────
#
# Output 40 bytes:
#   Offset  Size  Content
#   0       2  H  Remote I/O (R-IN)
#   2       2  H  Operation data number selection
#   4       2  H  Fixed I/O (IN)
#   6       2  H  Direct data operation type
#   8       4  i  Direct data operation position (steps, signed)
#   12      4  i  Direct data operation speed (Hz, signed)
#   16      4  i  Starting/changing rate (1 = 0.001 kHz/s)
#   20      4  i  Stopping deceleration  (1 = 0.001 kHz/s)
#   24      2  H  Operating current (1 = 0.1%, 1000 = 100%)
#   26      2  H  Forwarding destination (0=exec, 1=buffer)
#   28      2  H  Reserved
#   30      2  H  Read parameter ID
#   32      2  H  Write request
#   34      2  H  Write parameter ID
#   36      4  i  Write data
_OUT_FMT = struct.Struct("<HHHH iiii HHHHHH i")
assert _OUT_FMT.size == 40, f"Output assembly should be 40 bytes, got {_OUT_FMT.size}"

#
# Input 56 bytes:
#   Offset  Size  Content
#   0       2  H  Remote I/O (R-OUT)
#   2       2  H  Operation data number selection_R
#   4       2  H  Fixed I/O (OUT)
#   6       2  H  Present alarm code
#   8       4  i  Feedback position (steps, signed)
#   12      4  I  Feedback speed (Hz, unsigned)
#   16      4  i  Command position (steps, signed)
#   20      2  H  Torque monitor
#   22      2  H  CST operating current
#   24      4  I  Information
#   28      2  H  Reserved
#   30      2  H  Read parameter ID_R
#   32      2  H  Read/write status
#   34      2  H  Write parameter ID_R
#   36      4  i  Read data
#   40      4  i  Assignable monitor 0
#   44      4  i  Assignable monitor 1
#   48      4  i  Assignable monitor 2
#   52      4  i  Assignable monitor 3
_IN_FMT = struct.Struct("<HHHH iIi HH I HHHH iiiii")
assert _IN_FMT.size == 56, f"Input assembly should be 56 bytes, got {_IN_FMT.size}"

# CIP explicit messaging codes
_SVC_GET   = 0x0E   # Get Attribute Single
_SVC_SET   = 0x10   # Set Attribute Single
_CLS_ASM   = 0x04   # Assembly Object class
_ATTR_DATA = 3      # Data attribute


# ── Assembly builders / parsers ───────────────────────────────────────────────

def _build_output(r_in: int = 0,
                  op_num: int = 0,
                  fixed_io_in: int = 0,
                  dcmd_type: int = 0,
                  pos_steps: int = 0,
                  speed_hz: int = 0,
                  accel_raw: int = 0,
                  decel_raw: int = 0,
                  current_pct10: int = 1000,
                  fwd_dest: int = 0) -> bytes:
    return _OUT_FMT.pack(
        r_in, op_num, fixed_io_in, dcmd_type,
        pos_steps, speed_hz, accel_raw, decel_raw,
        current_pct10, fwd_dest,
        0, 0, 0, 0,   # reserved, read_param_id, write_req, write_param_id
        0,            # write data
    )


def _parse_input(data: bytes) -> dict | None:
    if len(data) < _IN_FMT.size:
        return None
    (r_out, _op_sel_r, fio_out, alarm_code,
     fb_pos, fb_speed_hz, _cmd_pos,
     _torque, _cst,
     _info,
     _reserved, _rd_param_r, _rw_status, _wr_param_r,
     _rd_data, *_monitors) = _IN_FMT.unpack_from(data)
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
        "fb_pos_steps": fb_pos,
        "fb_speed_hz":  fb_speed_hz,
    }


# ──────────────────────────────────────────────────────────────────────────────
# ROS2 node
# ──────────────────────────────────────────────────────────────────────────────

class EthernetIPMotorNode(Node):
    """
    Drop-in EtherNet/IP replacement for motor_controller_node.py.
    Same ROS2 topic API; communicates via CIP explicit messaging (pycomm3).
    """

    def __init__(self):
        super().__init__("ethernet_ip_motor")

        if not HAS_PYCOMM3:
            self.get_logger().fatal(
                "pycomm3 is not installed.  Run:  pip install pycomm3"
            )
            raise RuntimeError("pycomm3 required")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("ip_address",          "192.168.1.2")
        self.declare_parameter("motor_id",            "motor")
        self.declare_parameter("steps_per_mm",        100.0)
        self.declare_parameter("max_position_mm",      29.0)
        self.declare_parameter("min_speed_mms",         0.1)
        self.declare_parameter("max_speed_mms",       100.0)
        self.declare_parameter("acceleration_mms2",     0.0)
        self.declare_parameter("deceleration_mms2",     0.0)
        self.declare_parameter("poll_interval_s",       0.05)
        self.declare_parameter("dcmd_rdy_timeout_s",    3.0)

        self._ip           = self.get_parameter("ip_address").value
        self._motor_id     = self.get_parameter("motor_id").value
        self._spm          = self.get_parameter("steps_per_mm").value
        self._max_pos      = self.get_parameter("max_position_mm").value
        self._min_spd      = self.get_parameter("min_speed_mms").value
        self._max_spd      = self.get_parameter("max_speed_mms").value
        self._poll_dt      = self.get_parameter("poll_interval_s").value
        self._dcmd_timeout = self.get_parameter("dcmd_rdy_timeout_s").value

        acc_mms2 = self.get_parameter("acceleration_mms2").value
        dec_mms2 = self.get_parameter("deceleration_mms2").value
        # Convert mm/s² → raw units (1 raw = 0.001 kHz/s = 1 Hz/s)
        self._acc_raw = int(acc_mms2 * self._spm) if acc_mms2 > 0 else _DEFAULT_ACCEL_RAW
        self._dec_raw = int(dec_mms2 * self._spm) if dec_mms2 > 0 else _DEFAULT_ACCEL_RAW

        # ── Internal state ───────────────────────────────────────────────────
        self._driver     = None
        self._drv_lock   = threading.Lock()
        self._stop_event = threading.Event()
        self._state      = "idle"
        self._state_lock = threading.Lock()

        # Last motion command (kept so poll loop can re-send data if needed)
        self._last_pos_steps = 0
        self._last_speed_hz  = 0

        # ── ROS2 interface ───────────────────────────────────────────────────
        self.create_subscription(Float32MultiArray, "motor_cmd",  self._cmd_cb,  10)
        self.create_subscription(Empty,             "motor_home", self._home_cb, 10)
        self._status_pub = self.create_publisher(String, "motor_status", 10)

        # ── Connect & start poll thread ──────────────────────────────────────
        self._connect()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

        self._publish_status("idle")
        self.get_logger().info(
            f"[{self._motor_id}] EtherNet/IP motor node ready\n"
            f"  driver ip  : {self._ip}\n"
            f"  steps/mm   : {self._spm}\n"
            f"  pos  range : 0 – {self._max_pos} mm\n"
            f"  spd  range : {self._min_spd} – {self._max_spd} mm/s\n"
            "  topics     : motor_cmd / motor_home / motor_status"
        )

    # ── Connection management ──────────────────────────────────────────────────

    def _connect(self) -> bool:
        with self._drv_lock:
            if self._driver is not None:
                return True
            try:
                drv = CIPDriver(self._ip)
                drv.open()
                self._driver = drv
                self.get_logger().info(f"[{self._motor_id}] Connected to {self._ip}")
                return True
            except Exception as exc:
                self.get_logger().error(
                    f"[{self._motor_id}] EtherNet/IP connect failed: {exc}\n"
                    "  Check LAN cable, IP address, and driver power."
                )
                self._driver = None
                return False

    def _disconnect(self):
        with self._drv_lock:
            if self._driver is not None:
                try:
                    self._driver.close()
                except Exception:
                    pass
                self._driver = None

    def _ensure_connected(self) -> bool:
        with self._drv_lock:
            connected = self._driver is not None
        return connected or self._connect()

    # ── CIP helpers ────────────────────────────────────────────────────────────

    def _write_output(self, data: bytes) -> bool:
        """Set Attribute Single → output assembly instance 101, attribute 3."""
        with self._drv_lock:
            if self._driver is None:
                return False
            try:
                resp = self._driver.generic_message(
                    service=_SVC_SET,
                    class_code=_CLS_ASM,
                    instance=ASSEMBLY_OUTPUT,
                    attribute=_ATTR_DATA,
                    request_data=data,
                    connected=False,
                )
                if resp.error:
                    self.get_logger().error(
                        f"[{self._motor_id}] Write assembly error: {resp.error}"
                    )
                    return False
                return True
            except Exception as exc:
                self.get_logger().error(f"[{self._motor_id}] Write failed: {exc}")
                self._driver = None
                return False

    def _read_input(self) -> dict | None:
        """Get Attribute Single ← input assembly instance 100, attribute 3."""
        with self._drv_lock:
            if self._driver is None:
                return None
            try:
                resp = self._driver.generic_message(
                    service=_SVC_GET,
                    class_code=_CLS_ASM,
                    instance=ASSEMBLY_INPUT,
                    attribute=_ATTR_DATA,
                    connected=False,
                )
                if resp.error or resp.value is None:
                    return None
                return _parse_input(bytes(resp.value))
            except Exception as exc:
                self.get_logger().debug(f"[{self._motor_id}] Read failed: {exc}")
                self._driver = None
                return None

    # ── Motion commands ────────────────────────────────────────────────────────

    def _wait_dcmd_rdy(self) -> bool:
        """Block until DCMD-RDY is ON, or return False on timeout/alarm."""
        deadline = time.monotonic() + self._dcmd_timeout
        while time.monotonic() < deadline:
            if not self._ensure_connected():
                time.sleep(0.1)
                continue
            st = self._read_input()
            if st is None:
                time.sleep(0.05)
                continue
            if st["alarm"]:
                self.get_logger().error(
                    f"[{self._motor_id}] Alarm while waiting for DCMD-RDY: "
                    f"code=0x{st['alarm_code']:04x}"
                )
                return False
            if st["dcmd_rdy"]:
                return True
            time.sleep(0.02)
        self.get_logger().error(
            f"[{self._motor_id}] Timeout ({self._dcmd_timeout}s) waiting for DCMD-RDY"
        )
        return False

    def _move(self, pos_mm: float, spd_mms: float) -> bool:
        """
        Absolute positioning via direct data operation (TRIG ON-edge mode).
        Sequence per manual §6-3:
          1. Check DCMD-RDY ON
          2. Write operation data with TRIG = 0
          3. Set TRIG = 1  (rising edge → driver latches and starts)
          4. Clear TRIG = 0  (after driver acknowledges via TRIG_R)
        """
        pos_steps = int(round(pos_mm  * self._spm))
        speed_hz  = max(1, int(round(spd_mms * self._spm)))

        self._last_pos_steps = pos_steps
        self._last_speed_hz  = speed_hz

        # 1. Wait for DCMD-RDY
        if not self._wait_dcmd_rdy():
            return False

        # 2. Write data with TRIG = 0 (latch new data into driver)
        cmd_data = _build_output(
            dcmd_type=DCMD_ABS,
            pos_steps=pos_steps,
            speed_hz=speed_hz,
            accel_raw=self._acc_raw,
            decel_raw=self._dec_raw,
        )
        if not self._write_output(cmd_data):
            return False

        # Brief settle before rising edge
        time.sleep(0.001)

        # 3. Rising edge: TRIG = 1 (triggers the move)
        cmd_trig = _build_output(
            fixed_io_in=FIO_IN_TRIG,
            dcmd_type=DCMD_ABS,
            pos_steps=pos_steps,
            speed_hz=speed_hz,
            accel_raw=self._acc_raw,
            decel_raw=self._dec_raw,
        )
        if not self._write_output(cmd_trig):
            return False

        # 4. Clear TRIG (manual: "check TRIG_R ON, then turn TRIG OFF")
        #    Brief wait to ensure driver has seen the rising edge.
        time.sleep(0.005)
        cmd_clear = _build_output(
            fixed_io_in=0,   # TRIG = 0
            dcmd_type=DCMD_ABS,
            pos_steps=pos_steps,
            speed_hz=speed_hz,
            accel_raw=self._acc_raw,
            decel_raw=self._dec_raw,
        )
        return self._write_output(cmd_clear)

    def _home(self) -> bool:
        """High-speed return-to-home (ZHOME)."""
        # ZHOME ON — driver starts homing; poll loop detects HOME-END and clears it
        cmd = _build_output(fixed_io_in=FIO_IN_ZHOME)
        return self._write_output(cmd)

    def _send_stop(self):
        """Deceleration stop."""
        self._write_output(_build_output(fixed_io_in=FIO_IN_STOP))

    def _send_alarm_reset(self):
        """Reset alarm — pulse ALM-RST high then low."""
        self._write_output(_build_output(fixed_io_in=FIO_IN_ALM_RST))
        time.sleep(0.1)
        self._write_output(_build_output())   # clear

    def _clear_fio(self):
        """Write all control bits to 0."""
        self._write_output(_build_output())

    # ── ROS2 callbacks ─────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().error(
                f"[{self._motor_id}] motor_cmd needs [pos_mm, spd_mms]"
            )
            return

        pos_mm  = float(msg.data[0])
        spd_mms = float(msg.data[1])

        if not (0.0 <= pos_mm <= self._max_pos):
            self.get_logger().error(
                f"[{self._motor_id}] Position {pos_mm:.2f} mm out of range "
                f"[0, {self._max_pos}] — ignoring."
            )
            return
        if not (self._min_spd <= spd_mms <= self._max_spd):
            self.get_logger().error(
                f"[{self._motor_id}] Speed {spd_mms:.2f} mm/s out of range "
                f"[{self._min_spd}, {self._max_spd}] — ignoring."
            )
            return

        if not self._ensure_connected():
            self._publish_status("error")
            return

        self.get_logger().info(
            f"[{self._motor_id}] Move  pos={pos_mm:.2f} mm  spd={spd_mms:.2f} mm/s  "
            f"({int(pos_mm*self._spm)} steps  {int(spd_mms*self._spm)} Hz)"
        )
        self._publish_status("busy")

        if self._move(pos_mm, spd_mms):
            self._publish_status("running")
        else:
            self._publish_status("error")

    def _home_cb(self, _: Empty):
        self.get_logger().info(f"[{self._motor_id}] Homing command received (ZHOME).")

        if not self._ensure_connected():
            self._publish_status("error")
            return

        self._publish_status("busy")
        if self._home():
            self._publish_status("running")
        else:
            self._publish_status("error")

    # ── Status polling loop ────────────────────────────────────────────────────

    def _poll_loop(self):
        """
        Background thread: reads input assembly at poll_interval_s.

        State transitions:
          running → idle   when IN-POS or HOME-END goes high
          running → idle   when READY high and MOVE low (motion ended)
          running → error  when ALM-A, SET-ERR, or EXE-ERR goes high
          any     → error  when alarm detected while idle
        """
        prev_pos_r = None

        while not self._stop_event.is_set():
            with self._state_lock:
                state = self._state

            if state in ("running", "busy"):
                if not self._ensure_connected():
                    self._stop_event.wait(self._poll_dt)
                    continue

                st = self._read_input()
                if st is None:
                    self._stop_event.wait(self._poll_dt)
                    continue

                new_state = state

                if st["alarm"]:
                    new_state = "error"
                    self.get_logger().error(
                        f"[{self._motor_id}] ALARM  code=0x{st['alarm_code']:04x}"
                    )
                elif st["set_err"]:
                    new_state = "error"
                    self.get_logger().error(
                        f"[{self._motor_id}] Direct data SET-ERR (check op type / position)"
                    )
                elif st["exe_err"]:
                    new_state = "error"
                    self.get_logger().error(
                        f"[{self._motor_id}] Direct data EXE-ERR (execution failed)"
                    )
                elif st["in_pos"] or st["home_end"]:
                    new_state = "idle"
                    self._clear_fio()     # release ZHOME / control bits
                elif state == "running" and not st["move"] and st["ready"]:
                    # Motor stopped and driver is ready — treat as done
                    new_state = "idle"
                    self._clear_fio()

                # Log position during motion
                steps = st["fb_pos_steps"]
                pos_r = round(steps / self._spm, 1)
                if pos_r != prev_pos_r:
                    self.get_logger().info(
                        f"[{self._motor_id}]  fb_pos={pos_r:.1f} mm "
                        f"({steps} steps)  "
                        f"fb_spd={st['fb_speed_hz']/self._spm:.1f} mm/s  "
                        f"fio_out=0x{st['fio_out']:04x}"
                    )
                    prev_pos_r = pos_r

                if new_state != state:
                    self._publish_status(new_state)
                    prev_pos_r = None

            self._stop_event.wait(self._poll_dt)

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _publish_status(self, status: str):
        with self._state_lock:
            self._state = status
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f"[{self._motor_id}] status → {status}")

    # ── Cleanup ────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info(f"[{self._motor_id}] Shutting down.")
        self._stop_event.set()
        self._poll_thread.join(timeout=2.0)
        self._clear_fio()
        self._disconnect()
        super().destroy_node()


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = EthernetIPMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
