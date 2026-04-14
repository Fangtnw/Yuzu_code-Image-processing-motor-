import threading
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Float32, Empty, String


# ──────────────────────────────────────────────────────────────────────────────
# Motor configuration
# Add/uncomment entries as motors arrive — no other code changes needed.
#
#   type "linear"   → position (mm) + speed (mm/s) control  [DR28T style]
#   type "rotary"   → RPM + Spin / Stop buttons
#   type "conveyor" → single Step button
# ──────────────────────────────────────────────────────────────────────────────

MOTORS = [
    {"label": "Z Axis",          "ns": "motor_z",              "type": "linear"},
    {"label": "Yuzu Rotation",   "ns": "motor_yuzu_rot",       "type": "rotary"},
    {"label": "Peeler Orbit",    "ns": "motor_peeler_orbit",   "type": "rotary"},
    {"label": "Peeler Advance",  "ns": "motor_peeler_advance", "type": "linear"},
    {"label": "Conveyor",        "ns": "motor_conveyor",       "type": "conveyor"},
]

# ── Constants ─────────────────────────────────────────────────────────────────

MAX_POS   = 29.0
MIN_SPEED = 0.1
MAX_SPEED = 100.0
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

HARVEST_COLORS = {
    "idle":          "#9e9e9e",
    "feeding":       "#9c27b0",
    "lowering":      "#ff9800",
    "spinning_up":   "#2196f3",
    "peeling":       "#e91e63",
    "retracting":    "#ff9800",
    "spinning_down": "#2196f3",
    "raising":       "#ff9800",
    "done":          "#4caf50",
    "error":         "#f44336",
    "aborted":       "#795548",
}

HARVEST_STEPS = [
    "feeding", "lowering", "spinning_up", "peeling",
    "retracting", "spinning_down", "raising",
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

    def send_cmd(self, pos_mm: float, spd_mms: float):
        msg = Float32MultiArray()
        msg.data = [pos_mm, spd_mms]
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


class HarvestGuiNode(Node):
    """Talks to the harvest_sequence_node and listens for yuzu detections."""

    def __init__(self, status_callback, yuzu_callback):
        super().__init__("harvest_gui")
        self._start_pub = self.create_publisher(Empty, "/harvest/start", 10)
        self._abort_pub = self.create_publisher(Empty, "/harvest/abort", 10)
        self.create_subscription(String,             "/harvest/status",  lambda m: status_callback(m.data),   10)
        self.create_subscription(Float32MultiArray,  "/yuzu_detected",   lambda m: yuzu_callback(list(m.data)), 10)

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
    def __init__(self, parent: tk.Widget, node: MotorGuiNode, motor_type: str):
        self._node  = node
        self._type  = motor_type
        self._frame = ttk.Frame(parent)
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
        tk.Label(f, text=f"[0 – {MAX_POS}]", fg="gray").grid(row=0, column=2, sticky="w", padx=(6, 0))

        tk.Label(f, text="Speed (mm/s)").grid(row=1, column=0, sticky="w", pady=4)
        self._spd_var = tk.StringVar(value="5.0")
        ttk.Entry(f, textvariable=self._spd_var, width=10).grid(row=1, column=1, sticky="w", padx=(8, 0))
        tk.Label(f, text=f"[{MIN_SPEED} – {MAX_SPEED}]", fg="gray").grid(row=1, column=2, sticky="w", padx=(6, 0))

        ttk.Button(f, text="Send Command", command=self._on_send).grid(
            row=2, column=0, columnspan=3, pady=(10, 0), sticky="ew")

        hf = ttk.LabelFrame(self._frame, text="Homing", padding=12)
        hf.grid(row=2, column=0, columnspan=3, sticky="ew", **pad)
        ttk.Button(hf, text="Go Home", command=self._on_home).pack(fill="x")

    def _build_rotary(self, pad):
        f = ttk.LabelFrame(self._frame, text="Rotation Command", padding=12)
        f.grid(row=1, column=0, columnspan=3, sticky="ew", **pad)

        tk.Label(f, text="Speed (rpm)").grid(row=0, column=0, sticky="w", pady=4)
        self._rpm_var = tk.StringVar(value="200.0")
        ttk.Entry(f, textvariable=self._rpm_var, width=10).grid(row=0, column=1, sticky="w", padx=(8, 0))
        tk.Label(f, text=f"[0 – {MAX_RPM}]", fg="gray").grid(row=0, column=2, sticky="w", padx=(6, 0))

        bf = tk.Frame(f)
        bf.grid(row=1, column=0, columnspan=3, pady=(10, 0), sticky="ew")
        ttk.Button(bf, text="Spin", command=self._on_spin).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ttk.Button(bf, text="Stop", command=self._on_stop).pack(side="left", expand=True, fill="x")

        # Placeholder for row=2 so log stays at row=3
        tk.Frame(self._frame, height=1).grid(row=2)

    def _build_conveyor(self, pad):
        f = ttk.LabelFrame(self._frame, text="Conveyor", padding=12)
        f.grid(row=1, column=0, columnspan=3, sticky="ew", **pad)
        ttk.Button(f, text="Step  (advance one tray)", command=self._on_step).pack(fill="x")
        tk.Frame(self._frame, height=1).grid(row=2)

    # ── Handlers ──────────────────────────────────────────────────

    def _on_send(self):
        try:
            pos = float(self._pos_var.get().strip())
            spd = float(self._spd_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Input", "Position and speed must be numbers.")
            return
        if not (0.0 <= pos <= MAX_POS):
            messagebox.showerror("Out of Range", f"Position must be 0 – {MAX_POS} mm.")
            return
        if not (MIN_SPEED <= spd <= MAX_SPEED):
            messagebox.showerror("Out of Range", f"Speed must be {MIN_SPEED} – {MAX_SPEED} mm/s.")
            return
        self._node.send_cmd(pos, spd)
        _log_print(self._log, f">> motor_cmd  pos={pos} mm  speed={spd} mm/s")

    def _on_home(self):
        if not messagebox.askyesno("Confirm Homing", "Send homing command?"):
            return
        self._node.send_home()
        _log_print(self._log, ">> motor_home")

    def _on_spin(self):
        try:
            rpm = float(self._rpm_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Input", "RPM must be a number.")
            return
        if not (0.0 <= rpm <= MAX_RPM):
            messagebox.showerror("Out of Range", f"RPM must be 0 – {MAX_RPM}.")
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


# ── Harvest panel ─────────────────────────────────────────────────────────────

class HarvestPanel:
    def __init__(self, parent: tk.Widget, node: HarvestGuiNode):
        self._node  = node
        self._frame = ttk.Frame(parent)
        self._build_ui()

    @property
    def frame(self) -> ttk.Frame:
        return self._frame

    def _build_ui(self):
        pad = {"padx": 12, "pady": 5}

        # ── Harvest status ──
        hf = tk.Frame(self._frame, bg="#212121", pady=8)
        hf.grid(row=0, column=0, columnspan=3, sticky="ew")
        tk.Label(hf, text="Harvest:", bg="#212121", fg="white",
                 font=("Helvetica", 10)).pack(side="left", padx=(12, 6))
        self._h_dot = tk.Label(hf, text="●", bg="#212121",
                                fg=HARVEST_COLORS["idle"], font=("Helvetica", 14))
        self._h_dot.pack(side="left")
        self._h_lbl = tk.Label(hf, text="idle", bg="#212121", fg="white",
                                font=("Helvetica", 10, "bold"), width=14, anchor="w")
        self._h_lbl.pack(side="left", padx=(4, 0))

        # ── Yuzu detection (from image team) ──
        yf = ttk.LabelFrame(self._frame, text="Yuzu Detection  (/yuzu_detected)", padding=10)
        yf.grid(row=1, column=0, columnspan=3, sticky="ew", **pad)
        self._yuzu_dot = tk.Label(yf, text="●", fg="#9e9e9e", font=("Helvetica", 12))
        self._yuzu_dot.grid(row=0, column=0, padx=(0, 6))
        self._yuzu_lbl = tk.Label(yf, text="No detection yet", fg="gray", font=("Helvetica", 9))
        self._yuzu_lbl.grid(row=0, column=1, sticky="w")

        # ── Sequence progress ──
        pf = ttk.LabelFrame(self._frame, text="Sequence Progress", padding=10)
        pf.grid(row=2, column=0, columnspan=3, sticky="ew", **pad)
        self._step_dots  = {}
        self._step_lbls  = {}
        for i, step in enumerate(HARVEST_STEPS):
            row, col = divmod(i, 4)
            dot = tk.Label(pf, text="○", fg="#9e9e9e", font=("Helvetica", 10))
            dot.grid(row=row, column=col * 2, sticky="w", padx=(4, 2), pady=2)
            lbl = tk.Label(pf, text=step.replace("_", " "), fg="gray",
                           font=("Helvetica", 8), anchor="w")
            lbl.grid(row=row, column=col * 2 + 1, sticky="w", padx=(0, 12), pady=2)
            self._step_dots[step] = dot
            self._step_lbls[step] = lbl

        # ── Control buttons ──
        cf = ttk.LabelFrame(self._frame, text="Control", padding=10)
        cf.grid(row=3, column=0, columnspan=3, sticky="ew", **pad)
        ttk.Button(cf, text="Start Harvest", command=self._on_start).pack(
            side="left", expand=True, fill="x", padx=(0, 6))
        ttk.Button(cf, text="ABORT", command=self._on_abort).pack(
            side="left", expand=True, fill="x")

        # ── Log ──
        self._log = _log_widget(self._frame, row=4)
        _log_print(self._log, "Harvest panel ready.")
        _log_print(self._log, "Image team: publish /yuzu_detected then /harvest/start")
        _log_print(self._log, "  /yuzu_detected  [x_off_mm, y_off_mm, long_mm, short_mm]")

    # ── Handlers ──────────────────────────────────────────────────

    def _on_start(self):
        self._node.start()
        _log_print(self._log, ">> /harvest/start")

    def _on_abort(self):
        if not messagebox.askyesno("Confirm Abort", "Send ABORT to all motors?"):
            return
        self._node.abort()
        _log_print(self._log, ">> /harvest/abort")

    # ── Updates (called from ROS thread) ──────────────────────────

    def update_harvest_status(self, status: str, root: tk.Tk):
        root.after(0, self._apply_harvest_status, status)

    def _apply_harvest_status(self, status: str):
        color = HARVEST_COLORS.get(status, "white")
        self._h_dot.config(fg=color)
        self._h_lbl.config(text=status.replace("_", " "))
        _log_print(self._log, f"<< harvest: {status}")

        # Highlight current step; tick completed steps
        passed = True
        for step in HARVEST_STEPS:
            if step == status:
                self._step_dots[step].config(text="●", fg="#e91e63")
                self._step_lbls[step].config(fg="white", font=("Helvetica", 8, "bold"))
                passed = False
            elif passed:
                self._step_dots[step].config(text="✓", fg="#4caf50")
                self._step_lbls[step].config(fg="#4caf50", font=("Helvetica", 8))
            else:
                self._step_dots[step].config(text="○", fg="#9e9e9e")
                self._step_lbls[step].config(fg="gray", font=("Helvetica", 8))

        if status in ("idle", "done", "aborted", "error"):
            for step in HARVEST_STEPS:
                self._step_dots[step].config(text="○", fg="#9e9e9e")
                self._step_lbls[step].config(fg="gray", font=("Helvetica", 8))

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

        panel = MotorPanel(notebook, node, cfg["type"])
        panel_ref[0] = panel
        notebook.add(panel.frame, text=cfg["label"])

    # ── Harvest tab ──
    harvest_ref: list = [None]

    def harvest_status_cb(status):
        if harvest_ref[0]:
            harvest_ref[0].update_harvest_status(status, root)

    def yuzu_cb(data):
        if harvest_ref[0]:
            harvest_ref[0].update_yuzu_detected(data, root)

    harvest_node = HarvestGuiNode(harvest_status_cb, yuzu_cb)
    executor.add_node(harvest_node)
    nodes.append(harvest_node)

    harvest_panel = HarvestPanel(notebook, harvest_node)
    harvest_ref[0] = harvest_panel
    notebook.add(harvest_panel.frame, text="Harvest")

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
