import threading
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Empty, String


# ──────────────────────────────────────────────
# ROS2 Node (runs in background thread)
# ──────────────────────────────────────────────

class MotorGuiNode(Node):
    def __init__(self, status_callback):
        super().__init__("motor_gui")
        self._cmd_pub  = self.create_publisher(Float32MultiArray, "/motor_cmd",  10)
        self._home_pub = self.create_publisher(Empty,             "/motor_home", 10)
        self.create_subscription(String, "/motor_status", self._status_cb, 10)
        self._status_callback = status_callback

    def _status_cb(self, msg: String):
        self._status_callback(msg.data)

    def send_cmd(self, pos_mm: float, spd_mms: float):
        msg = Float32MultiArray()
        msg.data = [pos_mm, spd_mms]
        self._cmd_pub.publish(msg)
        self.get_logger().info(f"Published /motor_cmd: pos={pos_mm} mm, speed={spd_mms} mm/s")

    def send_home(self):
        self._home_pub.publish(Empty())
        self.get_logger().info("Published /motor_home")


# ──────────────────────────────────────────────
# GUI
# ──────────────────────────────────────────────

STATUS_COLORS = {
    "idle":    "#4caf50",   # green
    "busy":    "#ff9800",   # orange
    "running": "#2196f3",   # blue
    "error":   "#f44336",   # red
}

class MotorGUI:
    MAX_POS   = 29.0
    MIN_SPEED = 0.1

    def __init__(self, root: tk.Tk, node: MotorGuiNode):
        self._root = root
        self._node = node

        root.title("Motor Controller")
        root.resizable(False, False)

        self._build_ui()

    # ── UI Construction ────────────────────────

    def _build_ui(self):
        pad = {"padx": 12, "pady": 6}

        # ── Status bar ──
        status_frame = tk.Frame(self._root, bg="#212121", pady=8)
        status_frame.grid(row=0, column=0, columnspan=2, sticky="ew")

        tk.Label(status_frame, text="Motor Status:", bg="#212121",
                 fg="white", font=("Helvetica", 10)).pack(side="left", padx=(12, 6))

        self._status_dot = tk.Label(status_frame, text="●", bg="#212121",
                                    fg=STATUS_COLORS["idle"], font=("Helvetica", 14))
        self._status_dot.pack(side="left")

        self._status_label = tk.Label(status_frame, text="idle", bg="#212121",
                                      fg="white", font=("Helvetica", 10, "bold"), width=8, anchor="w")
        self._status_label.pack(side="left", padx=(4, 0))

        # ── Motion command section ──
        motion_frame = ttk.LabelFrame(self._root, text="Motion Command", padding=12)
        motion_frame.grid(row=1, column=0, columnspan=2, sticky="ew", **pad)

        tk.Label(motion_frame, text="Position (mm)").grid(row=0, column=0, sticky="w", pady=4)
        self._pos_var = tk.StringVar(value="10.0")
        pos_entry = ttk.Entry(motion_frame, textvariable=self._pos_var, width=12)
        pos_entry.grid(row=0, column=1, sticky="w", padx=(8, 0), pady=4)
        tk.Label(motion_frame, text=f"  [0 – {self.MAX_POS}]", fg="gray").grid(row=0, column=2, sticky="w")

        tk.Label(motion_frame, text="Speed (mm/s)").grid(row=1, column=0, sticky="w", pady=4)
        self._spd_var = tk.StringVar(value="3.0")
        spd_entry = ttk.Entry(motion_frame, textvariable=self._spd_var, width=12)
        spd_entry.grid(row=1, column=1, sticky="w", padx=(8, 0), pady=4)
        tk.Label(motion_frame, text=f"  [> {self.MIN_SPEED}]", fg="gray").grid(row=1, column=2, sticky="w")

        self._send_btn = ttk.Button(motion_frame, text="Send Command", command=self._on_send)
        self._send_btn.grid(row=2, column=0, columnspan=3, pady=(12, 0), sticky="ew")

        # ── Home section ──
        home_frame = ttk.LabelFrame(self._root, text="Homing", padding=12)
        home_frame.grid(row=2, column=0, columnspan=2, sticky="ew", **pad)

        self._home_btn = ttk.Button(home_frame, text="Go Home", command=self._on_home)
        self._home_btn.pack(fill="x")

        # ── Log box ──
        log_frame = ttk.LabelFrame(self._root, text="Log", padding=8)
        log_frame.grid(row=3, column=0, columnspan=2, sticky="nsew", **pad)
        self._root.rowconfigure(3, weight=1)
        self._root.columnconfigure(0, weight=1)

        self._log = tk.Text(log_frame, height=8, width=46, state="disabled",
                            bg="#1e1e1e", fg="#cccccc", font=("Courier", 9),
                            relief="flat", wrap="word")
        scrollbar = ttk.Scrollbar(log_frame, command=self._log.yview)
        self._log.configure(yscrollcommand=scrollbar.set)
        self._log.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self._log_print("GUI started. Waiting for /motor_status...")

    # ── Button handlers ────────────────────────

    def _on_send(self):
        pos_str = self._pos_var.get().strip()
        spd_str = self._spd_var.get().strip()

        try:
            pos = float(pos_str)
            spd = float(spd_str)
        except ValueError:
            messagebox.showerror("Invalid Input", "Position and speed must be numbers.")
            return

        if not (0.0 <= pos <= self.MAX_POS):
            messagebox.showerror("Out of Range", f"Position must be between 0 and {self.MAX_POS} mm.")
            return
        if spd <= self.MIN_SPEED:
            messagebox.showerror("Out of Range", f"Speed must be greater than {self.MIN_SPEED} mm/s.")
            return

        self._node.send_cmd(pos, spd)
        self._log_print(f">> /motor_cmd  pos={pos} mm  speed={spd} mm/s")

    def _on_home(self):
        if not messagebox.askyesno("Confirm Homing", "Send homing command?"):
            return
        self._node.send_home()
        self._log_print(">> /motor_home")

    # ── Status update (called from ROS thread) ─

    def update_status(self, status: str):
        self._root.after(0, self._apply_status, status)

    def _apply_status(self, status: str):
        color = STATUS_COLORS.get(status, "white")
        self._status_dot.config(fg=color)
        self._status_label.config(text=status)
        self._log_print(f"<< /motor_status: {status}")

    # ── Log helper ─────────────────────────────

    def _log_print(self, text: str):
        self._log.config(state="normal")
        self._log.insert("end", text + "\n")
        self._log.see("end")
        self._log.config(state="disabled")


# ──────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────

def main():
    rclpy.init()

    root = tk.Tk()

    # GUI created first so the status callback reference is ready
    gui_placeholder = [None]

    def status_callback(status: str):
        if gui_placeholder[0]:
            gui_placeholder[0].update_status(status)

    node = MotorGuiNode(status_callback)
    gui = MotorGUI(root, node)
    gui_placeholder[0] = gui

    # Spin ROS2 in a daemon thread so it stops when the window closes
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    def on_close():
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
