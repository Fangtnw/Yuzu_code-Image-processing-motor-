import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Float32, Empty, String


# ──────────────────────────────────────────────────────────────────────────────
# Motor configuration  (YuzuSequence.pdf)
#
#   type "linear"   → position (mm) + speed (mm/s) control  [DR28T / ABZO]
#   type "rotary"   → RPM + Spin / Stop  (add default_rpm and rpm_hint per spec)
#   type "conveyor" → single Step button
# ──────────────────────────────────────────────────────────────────────────────

MOTORS = [
    {
        "label": "Z Axis (M①)",
        "ns":    "motor_z",
        "type":  "linear",
    },
    {
        "label":       "Yuzu Rotation (M②)",
        "ns":          "motor_yuzu_rot",
        "type":        "rotary",
        "default_rpm": 225.0,
        "rpm_hint":    "spec: 200-250 rpm  CCW",
    },
    {
        "label": "Peeler Feed 3 (M③)",
        "ns":    "motor_peeler_3",
        "type":  "linear",
    },
    {
        "label": "Peeler Feed 4 (M④)",
        "ns":    "motor_peeler_4",
        "type":  "linear",
    },
    {
        "label":       "Peeler Orbit (M⑤)",
        "ns":          "motor_peeler_orbit",
        "type":        "rotary",
        "default_rpm": 20.0,
        "rpm_hint":    "spec: 20 rpm  CW (pos: 90° ROT)",
    },
    {
        "label": "Conveyor (M⑥)",
        "ns":    "motor_conveyor",
        "type":  "conveyor",
    },
]

# ── Constants ─────────────────────────────────────────────────────────────────

MAX_POS   = 29.0
MIN_SPEED = 0.1
MAX_SPEED = 100.0
MIN_ACCEL = 0.0
MAX_ACCEL = 100000.0
MAX_RPM   = 300.0

STATUS_COLORS = {
    "idle":     "#4caf50",
    "busy":     "#ff9800",
    "running":  "#2196f3",
    "spinning": "#2196f3",
    "stepping": "#9c27b0",
    "error":    "#f44336",
    "aborted":  "#795548",
    "done":     "#4caf50",
}

PEEL_COLORS = {
    "idle":               "#9e9e9e",
    "yuzu_positioning":   "#9c27b0",
    "yuzu_placement":     "#ff9800",
    "peeler_positioning": "#03a9f4",
    "rotation_feed":      "#e91e63",
    "yuzu_gripping":      "#f44336",
    "z_retracting":       "#ff9800",
    "yuzu_release":       "#4caf50",
    "done":               "#4caf50",
    "error":              "#f44336",
    "aborted":            "#795548",
}

# Ordered list of sequence states shown in the progress tracker
PEEL_STEPS = [
    "yuzu_positioning",
    "yuzu_placement",
    "peeler_positioning",
    "rotation_feed",
    "yuzu_gripping",
    "z_retracting",
    "yuzu_release",
]

# Human-readable label and which motors are active per step
PEEL_STEP_INFO = [
    ("① Yuzu positioning",   "M⑥ FD"),
    ("② Yuzu placement",     "M① Approach"),
    ("③ Peeler positioning", "M⑤ 90° ROT"),
    ("④ Rotation + feed",    "M② CCW → M③④ FD → M⑤ CW"),
    ("⑤ Yuzu gripping",      "M③④ FD"),
    ("⑥ Z retract",          "M① HOME  +stop M②⑤"),
    ("⑦ Yuzu release",       "M③④ HOME"),
]


# ──────────────────────────────────────────────────────────────────────────────
# ROS2 nodes
# ──────────────────────────────────────────────────────────────────────────────

class MotorGuiNode(Node):
    """One ROS2 node per motor, scoped to its namespace."""

    def __init__(self, namespace: str, motor_type: str, status_callback):
        node_name = "motor_gui_" + namespace.strip("/").replace("/", "_")
        super().__init__(node_name, namespace=namespace)
        self._type = motor_type

        if motor_type == "linear":
            self._cmd_pub  = self.create_publisher(Float32MultiArray, "motor_cmd",  10)
            self._home_pub = self.create_publisher(Empty,             "motor_home", 10)
        elif motor_type == "rotary":
            self._spin_pub = self.create_publisher(Float32, "motor_spin", 10)
            self._stop_pub = self.create_publisher(Empty,   "motor_stop", 10)
        elif motor_type == "conveyor":
            self._step_pub = self.create_publisher(Empty, "motor_step", 10)

        self.create_subscription(String, "motor_status", lambda m: status_callback(m.data), 10)

    def send_cmd(
        self,
        pos_mm: float,
        spd_mms: float,
        accel_mms2: float | None = None,
        decel_mms2: float | None = None,
        duration_s: float | None = None,
    ):
        msg = Float32MultiArray()
        msg.data = [pos_mm, spd_mms]
        if accel_mms2 is not None or decel_mms2 is not None or duration_s is not None:
            accel = 0.0 if accel_mms2 is None else accel_mms2
            decel = accel if decel_mms2 is None else decel_mms2
            duration = 0.0 if duration_s is None else duration_s
            msg.data.extend([accel, decel, duration])
        self._cmd_pub.publish(msg)

    def send_home(self):
        self._home_pub.publish(Empty())

    def send_spin(self, rpm: float):
        msg = Float32()
        msg.data = rpm
        self._spin_pub.publish(msg)

    def send_stop(self):
        self._stop_pub.publish(Empty())

    def send_step(self):
        self._step_pub.publish(Empty())


class PeelGuiNode(Node):
    """Talks to peel_sequence_node and listens for yuzu detections."""

    def __init__(self, status_callback, yuzu_callback):
        super().__init__("peel_gui")
        self._start_pub = self.create_publisher(Empty, "/peel/start", 10)
        self._abort_pub = self.create_publisher(Empty, "/peel/abort", 10)
        self.create_subscription(String,            "/peel/status",   lambda m: status_callback(m.data),     10)
        self.create_subscription(Float32MultiArray, "/yuzu_detected", lambda m: yuzu_callback(list(m.data)), 10)

    def start(self):
        self._start_pub.publish(Empty())

    def abort(self):
        self._abort_pub.publish(Empty())


# ──────────────────────────────────────────────────────────────────────────────
# GUI panels
# ──────────────────────────────────────────────────────────────────────────────

def _status_bar(parent, initial="idle") -> tuple:
    """Returns (dot_label, text_label). Reused across panels."""
    frame = tk.Frame(parent, bg="#212121", pady=8)
    frame.grid(row=0, column=0, columnspan=3, sticky="ew")
    tk.Label(frame, text="Status:", bg="#212121", fg="white",
             font=("Helvetica", 10)).pack(side="left", padx=(12, 6))
    dot = tk.Label(frame, text="●", bg="#212121",
                   fg=STATUS_COLORS.get(initial, "#9e9e9e"), font=("Helvetica", 14))
    dot.pack(side="left")
    lbl = tk.Label(frame, text=initial, bg="#212121", fg="white",
                   font=("Helvetica", 10, "bold"), width=12, anchor="w")
    lbl.pack(side="left", padx=(4, 0))
    return dot, lbl


def _log_widget(parent, row: int) -> tk.Text:
    """Returns a scrollable, read-only log Text widget."""
    lf = ttk.LabelFrame(parent, text="Log", padding=8)
    lf.grid(row=row, column=0, columnspan=3, sticky="nsew", padx=12, pady=6)
    parent.rowconfigure(row, weight=1)
    parent.columnconfigure(0, weight=1)
    log = tk.Text(lf, height=7, width=50, state="disabled",
                  bg="#1e1e1e", fg="#cccccc", font=("Courier", 9),
                  relief="flat", wrap="word")
    sb = ttk.Scrollbar(lf, command=log.yview)
    log.configure(yscrollcommand=sb.set)
    log.pack(side="left", fill="both", expand=True)
    sb.pack(side="right", fill="y")
    return log


def _log_print(log: tk.Text, text: str):
    log.config(state="normal")
    log.insert("end", text + "\n")
    log.see("end")
    log.config(state="disabled")


# ── Per-motor panel ───────────────────────────────────────────────────────────

class MotorPanel:
    def __init__(self, parent: tk.Widget, node: MotorGuiNode, cfg: dict):
        self._node        = node
        self._type        = cfg["type"]
        self._cfg         = cfg
        self._frame       = ttk.Frame(parent)
        self._last_pos_mm = 0.0
        self._build_ui()

    @property
    def frame(self) -> ttk.Frame:
        return self._frame

    def _build_ui(self):
        pad = {"padx": 12, "pady": 6}
        self._dot, self._lbl = _status_bar(self._frame)

        if self._type == "linear":
            self._build_linear(pad)
        elif self._type == "rotary":
            self._build_rotary(pad)
        elif self._type == "conveyor":
            self._build_conveyor(pad)

        self._log = _log_widget(self._frame, row=3)
        _log_print(self._log, "Panel ready.")

    def _build_linear(self, pad):
        f = ttk.LabelFrame(self._frame, text="Motion Command", padding=12)
        f.grid(row=1, column=0, columnspan=3, sticky="ew", **pad)

        tk.Label(f, text="Position (mm)").grid(row=0, column=0, sticky="w", pady=4)
        self._pos_var = tk.StringVar(value="10.0")
        ttk.Entry(f, textvariable=self._pos_var, width=10).grid(row=0, column=1, sticky="w", padx=(8, 0))
        tk.Label(f, text=f"[0 - {MAX_POS}]", fg="gray").grid(row=0, column=2, sticky="w", padx=(6, 0))

        tk.Label(f, text="Speed (mm/s)").grid(row=1, column=0, sticky="w", pady=4)
        self._spd_var = tk.StringVar(value="5.0")
        ttk.Entry(f, textvariable=self._spd_var, width=10).grid(row=1, column=1, sticky="w", padx=(8, 0))
        tk.Label(f, text=f"[{MIN_SPEED} - {MAX_SPEED}]", fg="gray").grid(row=1, column=2, sticky="w", padx=(6, 0))

        tk.Label(f, text="Time (s)").grid(row=2, column=0, sticky="w", pady=4)
        self._time_var = tk.StringVar(value="")
        ttk.Entry(f, textvariable=self._time_var, width=10).grid(row=2, column=1, sticky="w", padx=(8, 0))
        tk.Label(f, text="optional", fg="gray").grid(row=2, column=2, sticky="w", padx=(6, 0))

        tk.Label(f, text="Accel (mm/s²)").grid(row=3, column=0, sticky="w", pady=4)
        self._accel_var = tk.StringVar(value="50.0")
        ttk.Entry(f, textvariable=self._accel_var, width=10).grid(row=3, column=1, sticky="w", padx=(8, 0))
        tk.Label(f, text="USB timing only", fg="gray").grid(row=3, column=2, sticky="w", padx=(6, 0))

        ttk.Button(f, text="Send Command", command=self._on_send).grid(
            row=4, column=0, columnspan=3, pady=(10, 0), sticky="ew")

        hf = ttk.LabelFrame(self._frame, text="Homing", padding=12)
        hf.grid(row=2, column=0, columnspan=3, sticky="ew", **pad)
        ttk.Button(hf, text="Go Home", command=self._on_home).pack(fill="x")

    def _build_rotary(self, pad):
        default_rpm = self._cfg.get("default_rpm", 200.0)
        rpm_hint    = self._cfg.get("rpm_hint", f"0 - {MAX_RPM}")

        f = ttk.LabelFrame(self._frame, text="Rotation Command", padding=12)
        f.grid(row=1, column=0, columnspan=3, sticky="ew", **pad)

        tk.Label(f, text="Speed (rpm)").grid(row=0, column=0, sticky="w", pady=4)
        self._rpm_var = tk.StringVar(value=str(default_rpm))
        ttk.Entry(f, textvariable=self._rpm_var, width=10).grid(row=0, column=1, sticky="w", padx=(8, 0))
        tk.Label(f, text=rpm_hint, fg="gray").grid(row=0, column=2, sticky="w", padx=(6, 0))

        bf = tk.Frame(f)
        bf.grid(row=1, column=0, columnspan=3, pady=(10, 0), sticky="ew")
        ttk.Button(bf, text="Spin", command=self._on_spin).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ttk.Button(bf, text="Stop", command=self._on_stop).pack(side="left", expand=True, fill="x")

        tk.Frame(self._frame, height=1).grid(row=2)

    def _build_conveyor(self, pad):
        f = ttk.LabelFrame(self._frame, text="Conveyor", padding=12)
        f.grid(row=1, column=0, columnspan=3, sticky="ew", **pad)
        ttk.Button(f, text="Step  (advance one tray)", command=self._on_step).pack(fill="x")
        tk.Frame(self._frame, height=1).grid(row=2)

    # ── Handlers ──────────────────────────────────────────────────

    def _on_send(self):
        try:
            pos   = float(self._pos_var.get().strip())
            spd   = float(self._spd_var.get().strip())
            accel = float(self._accel_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Input", "Position, speed, and acceleration must be numbers.")
            return

        time_text = self._time_var.get().strip()
        duration  = None
        if time_text:
            try:
                duration = float(time_text)
            except ValueError:
                messagebox.showerror("Invalid Input", "Time must be a number.")
                return

        if not (0.0 <= pos <= MAX_POS):
            messagebox.showerror("Out of Range", f"Position must be 0 - {MAX_POS} mm.")
            return
        if not (MIN_ACCEL <= accel <= MAX_ACCEL):
            messagebox.showerror("Out of Range", f"Acceleration must be {MIN_ACCEL} - {MAX_ACCEL} mm/s².")
            return

        if duration is not None:
            if duration <= 0.0:
                messagebox.showerror("Out of Range", "Time must be > 0 s.")
                return
            spd = self._speed_for_duration(abs(pos - self._last_pos_mm), duration, accel)
            if spd is None:
                return

        if not (MIN_SPEED <= spd <= MAX_SPEED):
            messagebox.showerror("Out of Range", f"Speed must be {MIN_SPEED} - {MAX_SPEED} mm/s.")
            return

        if duration is None:
            self._node.send_cmd(pos, spd)
        else:
            self._node.send_cmd(pos, spd, accel, accel, duration)
        self._last_pos_mm = pos

        log_line = f">> motor_cmd  pos={pos} mm  speed={spd:.3f} mm/s"
        if duration is not None:
            log_line += f"  accel={accel:g} mm/s²  time={duration:g} s"
        _log_print(self._log, log_line)

    def _speed_for_duration(self, distance_mm: float, duration_s: float, accel_mms2: float) -> float | None:
        if distance_mm <= 0.0:
            return MIN_SPEED
        if accel_mms2 <= 0.0:
            return distance_mm / duration_s

        min_time_s = 2.0 * math.sqrt(distance_mm / accel_mms2)
        if duration_s < min_time_s:
            messagebox.showerror(
                "Time Too Short",
                f"This move needs at least {min_time_s:.3f} s at {accel_mms2:g} mm/s².",
            )
            return None

        disc = accel_mms2 * accel_mms2 * duration_s * duration_s - 4.0 * accel_mms2 * distance_mm
        if disc <= 0.0:
            return math.sqrt(distance_mm * accel_mms2)
        return (accel_mms2 * duration_s - math.sqrt(disc)) / 2.0

    def _on_home(self):
        if not messagebox.askyesno("Confirm Homing", "Send homing command?"):
            return
        self._node.send_home()
        self._last_pos_mm = 0.0
        _log_print(self._log, ">> motor_home")

    def _on_spin(self):
        try:
            rpm = float(self._rpm_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Input", "RPM must be a number.")
            return
        if not (0.0 <= rpm <= MAX_RPM):
            messagebox.showerror("Out of Range", f"RPM must be 0 - {MAX_RPM}.")
            return
        self._node.send_spin(rpm)
        _log_print(self._log, f">> motor_spin  rpm={rpm}")

    def _on_stop(self):
        self._node.send_stop()
        _log_print(self._log, ">> motor_stop")

    def _on_step(self):
        self._node.send_step()
        _log_print(self._log, ">> motor_step")

    # ── Status update (called from ROS thread) ─────────────────────

    def update_status(self, status: str, root: tk.Tk):
        root.after(0, self._apply_status, status)

    def _apply_status(self, status: str):
        color = STATUS_COLORS.get(status, "white")
        self._dot.config(fg=color)
        self._lbl.config(text=status)
        _log_print(self._log, f"<< status: {status}")


# ── Peel Sequence panel ───────────────────────────────────────────────────────

class PeelPanel:
    def __init__(self, parent: tk.Widget, node: PeelGuiNode):
        self._node  = node
        self._frame = ttk.Frame(parent)
        self._build_ui()

    @property
    def frame(self) -> ttk.Frame:
        return self._frame

    def _build_ui(self):
        pad = {"padx": 12, "pady": 5}

        # ── Peel sequence status header ──
        hf = tk.Frame(self._frame, bg="#212121", pady=8)
        hf.grid(row=0, column=0, columnspan=3, sticky="ew")
        tk.Label(hf, text="Peel:", bg="#212121", fg="white",
                 font=("Helvetica", 10)).pack(side="left", padx=(12, 6))
        self._h_dot = tk.Label(hf, text="●", bg="#212121",
                                fg=PEEL_COLORS["idle"], font=("Helvetica", 14))
        self._h_dot.pack(side="left")
        self._h_lbl = tk.Label(hf, text="idle", bg="#212121", fg="white",
                                font=("Helvetica", 10, "bold"), width=18, anchor="w")
        self._h_lbl.pack(side="left", padx=(4, 0))

        # ── Yuzu detection (from image team) ──
        yf = ttk.LabelFrame(self._frame, text="Yuzu Detection  (/yuzu_detected)", padding=10)
        yf.grid(row=1, column=0, columnspan=3, sticky="ew", **pad)
        self._yuzu_dot = tk.Label(yf, text="●", fg="#9e9e9e", font=("Helvetica", 12))
        self._yuzu_dot.grid(row=0, column=0, padx=(0, 6))
        self._yuzu_lbl = tk.Label(yf, text="No detection yet", fg="gray", font=("Helvetica", 9))
        self._yuzu_lbl.grid(row=0, column=1, sticky="w")

        # ── Sequence progress — one row per step ──
        pf = ttk.LabelFrame(self._frame, text="Sequence Progress  (YuzuSequence.pdf)", padding=10)
        pf.grid(row=2, column=0, columnspan=3, sticky="ew", **pad)
        pf.columnconfigure(2, weight=1)

        self._step_dots = {}
        self._step_lbls = {}
        for i, (step, (label, motors)) in enumerate(zip(PEEL_STEPS, PEEL_STEP_INFO)):
            dot = tk.Label(pf, text="○", fg="#9e9e9e", font=("Helvetica", 10))
            dot.grid(row=i, column=0, sticky="w", padx=(4, 6), pady=1)

            name_lbl = tk.Label(pf, text=label, fg="gray",
                                 font=("Helvetica", 8, "bold"), anchor="w", width=22)
            name_lbl.grid(row=i, column=1, sticky="w", pady=1)

            motor_lbl = tk.Label(pf, text=motors, fg="#666666",
                                  font=("Courier", 8), anchor="w")
            motor_lbl.grid(row=i, column=2, sticky="w", padx=(4, 0), pady=1)

            self._step_dots[step] = dot
            self._step_lbls[step] = name_lbl

        # ── Control buttons ──
        cf = ttk.LabelFrame(self._frame, text="Control", padding=10)
        cf.grid(row=3, column=0, columnspan=3, sticky="ew", **pad)
        ttk.Button(cf, text="Start Peel", command=self._on_start).pack(
            side="left", expand=True, fill="x", padx=(0, 6))
        ttk.Button(cf, text="ABORT", command=self._on_abort).pack(
            side="left", expand=True, fill="x")

        # ── Log ──
        self._log = _log_widget(self._frame, row=4)
        _log_print(self._log, "Peel sequence panel ready.  6-motor / 8-step.")
        _log_print(self._log, "Image team interface:")
        _log_print(self._log, "  /yuzu_detected  [x_off_mm, y_off_mm, long_mm, short_mm]")
        _log_print(self._log, "  /peel/start  →  triggers cycle")
        _log_print(self._log, "  /peel/abort  →  emergency stop all motors")

    # ── Handlers ──────────────────────────────────────────────────

    def _on_start(self):
        self._node.start()
        _log_print(self._log, ">> /peel/start")

    def _on_abort(self):
        if not messagebox.askyesno("Confirm Abort", "Send ABORT to all motors?"):
            return
        self._node.abort()
        _log_print(self._log, ">> /peel/abort")

    # ── Updates (called from ROS thread) ──────────────────────────

    def update_peel_status(self, status: str, root: tk.Tk):
        root.after(0, self._apply_peel_status, status)

    def _apply_peel_status(self, status: str):
        color = PEEL_COLORS.get(status, "white")
        self._h_dot.config(fg=color)
        self._h_lbl.config(text=status.replace("_", " "))
        _log_print(self._log, f"<< peel: {status}")

        # Reset all to idle appearance on terminal states
        if status in ("idle", "done", "aborted", "error"):
            for step in PEEL_STEPS:
                self._step_dots[step].config(text="○", fg="#9e9e9e")
                self._step_lbls[step].config(fg="gray", font=("Helvetica", 8, "bold"))
            return

        # Tick completed steps, highlight current, grey out pending
        passed = True
        for step in PEEL_STEPS:
            if step == status:
                self._step_dots[step].config(text="●", fg="#e91e63")
                self._step_lbls[step].config(fg="white", font=("Helvetica", 8, "bold"))
                passed = False
            elif passed:
                self._step_dots[step].config(text="✓", fg="#4caf50")
                self._step_lbls[step].config(fg="#4caf50", font=("Helvetica", 8, "bold"))
            else:
                self._step_dots[step].config(text="○", fg="#9e9e9e")
                self._step_lbls[step].config(fg="gray", font=("Helvetica", 8, "bold"))

    def update_yuzu_detected(self, data: list, root: tk.Tk):
        root.after(0, self._apply_yuzu, data)

    def _apply_yuzu(self, data: list):
        if len(data) >= 4:
            self._yuzu_dot.config(fg="#4caf50")
            self._yuzu_lbl.config(
                text=(f"long={data[2]:.1f} mm  short={data[3]:.1f} mm  "
                      f"offset=({data[0]:+.1f}, {data[1]:+.1f}) mm"),
                fg="white",
            )
            _log_print(self._log,
                       f"<< yuzu_detected: long={data[2]:.1f}  short={data[3]:.1f} mm")


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()

    root = tk.Tk()
    root.title("Yuzu Peeler — Motor Controller")
    root.resizable(False, False)

    executor = MultiThreadedExecutor()
    nodes: list[Node] = []

    notebook = ttk.Notebook(root)
    notebook.pack(fill="both", expand=True, padx=8, pady=8)

    # ── One tab per motor ──
    for cfg in MOTORS:
        panel_ref: list = [None]

        def make_status_cb(ref, r):
            def cb(status):
                if ref[0]:
                    ref[0].update_status(status, r)
            return cb

        node = MotorGuiNode(
            namespace=cfg["ns"],
            motor_type=cfg["type"],
            status_callback=make_status_cb(panel_ref, root),
        )
        executor.add_node(node)
        nodes.append(node)

        panel = MotorPanel(notebook, node, cfg)
        panel_ref[0] = panel
        notebook.add(panel.frame, text=cfg["label"])

    # ── Peel sequence tab ──
    peel_ref: list = [None]

    def peel_status_cb(status):
        if peel_ref[0]:
            peel_ref[0].update_peel_status(status, root)

    def yuzu_cb(data):
        if peel_ref[0]:
            peel_ref[0].update_yuzu_detected(data, root)

    peel_node = PeelGuiNode(peel_status_cb, yuzu_cb)
    executor.add_node(peel_node)
    nodes.append(peel_node)

    peel_panel = PeelPanel(notebook, peel_node)
    peel_ref[0] = peel_panel
    notebook.add(peel_panel.frame, text="Peel Sequence")

    # ── Spin all ROS2 nodes in one background thread ──
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    def on_close():
        executor.shutdown(timeout_sec=1.0)
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
