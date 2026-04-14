import os
import re
import time
import threading
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Empty, String
from ament_index_python.packages import get_package_share_directory


# ──────────────────────────────────────────────
# Data file helpers
# ──────────────────────────────────────────────

def _data_path(filename: str) -> str:
    """Return the absolute path to a data file installed with this package."""
    share_dir = get_package_share_directory("motor_controller")
    return os.path.join(share_dir, "data", filename)


# ──────────────────────────────────────────────
# Helper functions (ported from playback2.py)
# ──────────────────────────────────────────────

def create_combined_html(target_position_mm, target_speed_mms):
    # ==========================================
    # 1. POSITION CALCULATION (Baseline: 25mm)
    # ==========================================
    pos_base = [0xa8, 0x61, 0x00, 0x00]
    pos_base_cs = [0xb9, 0xf9, 0x85]

    pos_um = int(target_position_mm * 1000)
    pos_target = [
        (pos_um & 0xFF), ((pos_um >> 8) & 0xFF),
        ((pos_um >> 16) & 0xFF), ((pos_um >> 24) & 0xFF)
    ]

    pos_mask = (pos_base[0] ^ pos_base[1] ^ pos_base[2] ^ pos_base[3]) ^ \
               (pos_target[0] ^ pos_target[1] ^ pos_target[2] ^ pos_target[3])
    pos_cs = [cs ^ pos_mask for cs in pos_base_cs]

    try:
        with open(_data_path("write25mm.html"), "r", encoding="latin-1") as f:
            pos_html = f.read()

        pos_html = pos_html.replace(
            "01 0c a8 61 00 00",
            f"01 0c {pos_target[0]:02x} {pos_target[1]:02x} {pos_target[2]:02x} {pos_target[3]:02x}"
        )
        pos_html = pos_html.replace("81 0c 00 00 00 00 00 b9", f"81 0c 00 00 00 00 00 {pos_cs[0]:02x}")
        pos_html = pos_html.replace(
            "01 0f 00 00 00 00 00 f9 00 00 00 00 00 00 00 85",
            f"01 0f 00 00 00 00 00 {pos_cs[1]:02x} 00 00 00 00 00 00 00 {pos_cs[2]:02x}"
        )
    except FileNotFoundError:
        return None

    # ==========================================
    # 2. SPEED CALCULATION (Baseline: 1 mm/s)
    # ==========================================
    spd_base = [0xe8, 0x03, 0x00, 0x00]
    spd_base_cs = [0x5a, 0x58, 0xa5]

    spd_um = int(target_speed_mms * 1000)
    spd_target = [
        (spd_um & 0xFF), ((spd_um >> 8) & 0xFF),
        ((spd_um >> 16) & 0xFF), ((spd_um >> 24) & 0xFF)
    ]

    spd_mask = (spd_base[0] ^ spd_base[1] ^ spd_base[2] ^ spd_base[3]) ^ \
               (spd_target[0] ^ spd_target[1] ^ spd_target[2] ^ spd_target[3])
    spd_cs = [cs ^ spd_mask for cs in spd_base_cs]

    try:
        with open(_data_path("1mms.html"), "r", encoding="latin-1") as f:
            spd_html = f.read()

        spd_html = spd_html.replace("02 0c e8 03", f"02 0c {spd_target[0]:02x} {spd_target[1]:02x}")
        spd_html = spd_html.replace("00 00 22 0c e8 03", f"{spd_target[2]:02x} {spd_target[3]:02x} 22 0c e8 03")
        spd_html = spd_html.replace("42 0c e8 03 00 00 00 5a", f"42 0c e8 03 00 00 00 {spd_cs[0]:02x}")
        spd_html = spd_html.replace(
            "e8 03 00 00 00 58 00 00 00 00 00 00 00 00 00 a5",
            f"e8 03 00 00 00 {spd_cs[1]:02x} 00 00 00 00 00 00 00 00 00 {spd_cs[2]:02x}"
        )
    except FileNotFoundError:
        return None

    # ==========================================
    # 3. COMBINE FILES
    # ==========================================
    spd_table_content = spd_html.split("<table>")[1].split("</table>")[0]

    marker = "Written data (COM3)"
    first_idx = spd_table_content.find(marker)
    second_idx = spd_table_content.find(marker, first_idx + 1)

    if second_idx != -1:
        tr_start = spd_table_content.rfind("<tr", 0, second_idx)
        spd_table_content = spd_table_content[tr_start:]

    divider = '\n<tr class="s3"><td colspan="2"><br/><b>--- SPEED COMMAND START ---</b><br/></td></tr>\n'
    combined_html = pos_html.replace("</table>", divider + spd_table_content + "\n</table>")

    return combined_html


def parse_html_content(html_content):
    """Parses MEXE02 HTML text and extracts all written hex payloads into bytearrays."""
    payloads = []
    current_payload = bytearray()
    lines = html_content.splitlines()

    for line in lines:
        if "Written data" in line:
            if current_payload:
                payloads.append(current_payload)
                current_payload = bytearray()
        elif 'class="s1"' in line and '<pre>' in line:
            match = re.search(r'<pre>(.*?)</pre>', line)
            if match:
                pre_text = match.group(1)
                hex_part = pre_text[:48]
                for b in hex_part.split():
                    if len(b) == 2:
                        try:
                            current_payload.append(int(b, 16))
                        except ValueError:
                            pass

    if current_payload:
        payloads.append(current_payload)

    return payloads


# ──────────────────────────────────────────────
# ROS2 Node
# ──────────────────────────────────────────────

class MotorControllerNode(Node):

    def __init__(self):
        super().__init__("motor_controller")

        # ── Declare ROS2 parameters (hardcoded defaults) ──
        self.declare_parameter("com_port",            "/dev/ttyACM0")
        self.declare_parameter("baudrate",            19200)
        self.declare_parameter("serial_timeout",      0.5)
        self.declare_parameter("serial_write_timeout",0.5)
        self.declare_parameter("max_position_mm",     29.0)
        self.declare_parameter("frame_delay",         0.1)
        self.declare_parameter("heartbeat_interval",  0.2)
        self.declare_parameter("motor_id",            "motor")

        com_port             = self.get_parameter("com_port").value
        baudrate             = self.get_parameter("baudrate").value
        serial_timeout       = self.get_parameter("serial_timeout").value
        serial_write_timeout = self.get_parameter("serial_write_timeout").value
        self._max_pos        = self.get_parameter("max_position_mm").value
        self._frame_delay    = self.get_parameter("frame_delay").value
        self._hb_interval    = self.get_parameter("heartbeat_interval").value
        self._motor_id       = self.get_parameter("motor_id").value

        self._serial_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._heartbeat_thread = None

        # Open serial port once for the lifetime of the node
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
            self.get_logger().info(f"[{self._motor_id}] Opened {com_port} successfully.")
        except serial.SerialException as e:
            self.get_logger().fatal(f"[{self._motor_id}] Failed to open {com_port}: {e}")
            raise

        self.create_subscription(Float32MultiArray, "/motor_cmd", self._cmd_callback, 10)
        self.create_subscription(Empty, "/motor_home", self._home_callback, 10)
        self._status_pub = self.create_publisher(String, "/motor_status", 10)
        self._publish_status("idle")

        self.get_logger().info(
            "Motor controller node ready.\n"
            "  /motor_cmd  [Float32MultiArray]  data[0]=position_mm, data[1]=speed_mms\n"
            "  /motor_home [Empty]              trigger homing sequence"
        )

    # ── Callbacks ──────────────────────────────

    def _cmd_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().error("Expected data[0]=position_mm, data[1]=speed_mms — ignoring.")
            return

        pos_mm = float(msg.data[0])
        spd_mms = float(msg.data[1])

        if not (0.0 <= pos_mm <= self._max_pos):
            self.get_logger().error(f"Position {pos_mm} mm out of range [0, {self._max_pos}] — ignoring.")
            return
        if spd_mms <= 0.0:
            self.get_logger().error(f"Speed must be > 0 mm/s — ignoring.")
            return

        self.get_logger().info(f"Received command: pos={pos_mm} mm, speed={spd_mms} mm/s")

        config_html = create_combined_html(pos_mm, spd_mms)
        if config_html is None:
            self.get_logger().error("Failed to generate HTML — check that write25mm.html and 1mms.html exist in the package share data/.")
            self._publish_status("error")
            return

        payloads = parse_html_content(config_html)

        try:
            with open(_data_path("start_run.html"), "r", encoding="latin-1", errors="ignore") as f:
                payloads.extend(parse_html_content(f.read()))
        except FileNotFoundError:
            self.get_logger().warn("start_run.html not found — motor configured but will NOT execute motion.")

        self._execute_command(payloads)

    def _home_callback(self, _msg: Empty):
        self.get_logger().info("Homing command received.")
        try:
            with open(_data_path("2home.html"), "r", encoding="latin-1", errors="ignore") as f:
                payloads = parse_html_content(f.read())
        except FileNotFoundError:
            self.get_logger().error("2home.html not found — cannot home.")
            self._publish_status("error")
            return

        self._execute_command(payloads)

    # ── Status ─────────────────────────────────

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f"Motor status: {status}")

    # ── Execution & Heartbeat ──────────────────

    def _execute_command(self, payloads):
        """Stop current heartbeat, send payloads, restart heartbeat."""
        self._stop_heartbeat()
        self._publish_status("busy")

        ping_command = None
        with self._serial_lock:
            self.get_logger().info(f"Sending {len(payloads)} frames...")
            for i, payload in enumerate(payloads):
                self.get_logger().info(f"[Frame {i+1}/{len(payloads)}] {payload.hex(' ')}")
                self._ser.write(payload)
                time.sleep(self._frame_delay)
                if self._ser.in_waiting > 0:
                    resp = self._ser.read(self._ser.in_waiting)
                    self.get_logger().info(f"  Response: {resp.hex(' ')}")

                if ping_command is None and len(payload) > 1 and payload[1] == 0x03:
                    ping_command = payload

        if ping_command is None and payloads:
            ping_command = payloads[0]

        self.get_logger().info("All frames sent. Starting heartbeat...")
        self._start_heartbeat(ping_command)
        self._publish_status("running")

    def _start_heartbeat(self, ping_command):
        self._stop_event.clear()
        self._heartbeat_thread = threading.Thread(
            target=self._run_heartbeat, args=(ping_command,), daemon=True
        )
        self._heartbeat_thread.start()

    def _stop_heartbeat(self):
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._stop_event.set()
            self._heartbeat_thread.join(timeout=2.0)

    def _run_heartbeat(self, ping_command):
        ping_count = 0
        while not self._stop_event.is_set():
            if ping_command:
                with self._serial_lock:
                    try:
                        self._ser.write(ping_command)
                    except serial.SerialException:
                        break
                ping_count += 1
                if ping_count % 5 == 0:
                    self.get_logger().info(f"Motor alive... {ping_count} pings sent.", throttle_duration_sec=1.0)
            self._stop_event.wait(timeout=self._hb_interval)

    # ── Cleanup ────────────────────────────────

    def destroy_node(self):
        self.get_logger().info("Shutting down — stopping heartbeat and closing serial port.")
        self._stop_heartbeat()
        self._publish_status("idle")
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


# ──────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────

def main():
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
