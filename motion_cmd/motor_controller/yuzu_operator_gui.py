"""Simple guarded operator panel for the Yuzu peeler commissioning system."""

import math
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64, Float64MultiArray


AXIS1_TOPIC = "/axis1_position_controller/commands"
AXIS2_TOPIC = "/axis2_velocity_controller/commands_rpm"
MOTOR3_TOPIC = "/motor3_position_controller/commands_mm"
MOTOR4_TOPIC = "/motor4_position_controller/commands_mm"
MOTOR5_TOPIC = "/motor5_position_controller/commands_deg"
MOTOR5_SET_ZERO_TOPIC = "/motor5_position_controller/set_zero"
MOTOR6_TOPIC = "/motor6_conveyor/commands_rpm"
AXIS1_MIN_MM = 0.0996
AXIS1_MAX_MM = 200.0
AXIS1_INCREMENT_MM = 0.0012
AXIS1_PARK_MM = 0.0996
AXIS1_VELOCITY_MM_S = 15.0
AXIS1_ACCELERATION_MM_S2 = 15.0
AXIS2_GUI_MAX_RPM = 250.0
AXIS2_ACCELERATION_RPM_S = 25.0
MOTOR4_MIN_MM = 0.0
MOTOR4_MAX_MM = 15.0
MOTOR4_INCREMENT_MM = 0.001
MOTOR3_MIN_MM = 0.0
MOTOR3_MAX_MM = 15.0
MOTOR3_INCREMENT_MM = 0.001
MOTOR3_4_VELOCITY_MM_S = 8.0
MOTOR3_ACCELERATION_MM_S2 = 50.0
MOTOR4_ACCELERATION_MM_S2 = 50.0
MOTOR5_MAX_ANGLE_DEG = 90.0
MOTOR5_VELOCITY_RPM = 20.0
MOTOR5_ACCELERATION_RPM_S = 20.0
MOTOR6_GUI_MAX_RPM = 50.0
MOTOR6_ACCELERATION_RPM_S = 25.0
MOTOR6_PULLEY_DIAMETER_MM = 30.0
MOTOR6_GUI_MAX_MM_S = MOTOR6_GUI_MAX_RPM * math.pi * MOTOR6_PULLEY_DIAMETER_MM / 60.0
FEEDBACK_STALE_S = 2.0


def aligned_axis1_position_m(requested_mm: float) -> float:
    """Validate and align an operator position to one Axis 1 encoder count."""
    if not math.isfinite(requested_mm):
        raise ValueError("Motor 1 position must be a finite number")
    if not AXIS1_MIN_MM <= requested_mm <= AXIS1_MAX_MM:
        raise ValueError(
            f"Motor 1 position must be {AXIS1_MIN_MM:.4f}..{AXIS1_MAX_MM:.1f} mm"
        )
    counts = round(requested_mm / AXIS1_INCREMENT_MM)
    aligned_mm = counts * AXIS1_INCREMENT_MM
    if not AXIS1_MIN_MM <= aligned_mm <= AXIS1_MAX_MM:
        raise ValueError("Aligned Motor 1 position is outside the guarded range")
    return aligned_mm / 1000.0


def checked_axis2_rpm(requested_rpm: float) -> float:
    """Validate an operator RPM against the GUI commissioning ceiling."""
    if not math.isfinite(requested_rpm):
        raise ValueError("Motor 2 speed must be a finite number")
    if not 0.0 <= requested_rpm <= AXIS2_GUI_MAX_RPM:
        raise ValueError(f"Motor 2 speed must be 0..{AXIS2_GUI_MAX_RPM:.1f} rpm")
    return requested_rpm


def aligned_motor4_position_mm(requested_mm: float) -> float:
    """Validate and align Motor 4 to its 0.001 mm command increment."""
    if not math.isfinite(requested_mm):
        raise ValueError("Motor 4 position must be a finite number")
    if not MOTOR4_MIN_MM <= requested_mm <= MOTOR4_MAX_MM:
        raise ValueError("Motor 4 position must be 0..15 mm")
    aligned_mm = round(requested_mm / MOTOR4_INCREMENT_MM) * MOTOR4_INCREMENT_MM
    if not MOTOR4_MIN_MM <= aligned_mm <= MOTOR4_MAX_MM:
        raise ValueError("Aligned Motor 4 position is outside the guarded range")
    return aligned_mm


def aligned_motor3_position_mm(requested_mm: float) -> float:
    """Validate and align Motor 3 to its 0.001 mm command increment."""
    if not math.isfinite(requested_mm):
        raise ValueError("Motor 3 position must be a finite number")
    if not MOTOR3_MIN_MM <= requested_mm <= MOTOR3_MAX_MM:
        raise ValueError("Motor 3 position must be 0..15 mm")
    aligned_mm = round(requested_mm / MOTOR3_INCREMENT_MM) * MOTOR3_INCREMENT_MM
    if not MOTOR3_MIN_MM <= aligned_mm <= MOTOR3_MAX_MM:
        raise ValueError("Aligned Motor 3 position is outside the guarded range")
    return aligned_mm


def checked_motor6_speed_mm_s(requested_mm_s: float) -> float:
    """Validate belt speed and convert it to gearbox-output RPM."""
    if not math.isfinite(requested_mm_s):
        raise ValueError("Motor 6 belt speed must be a finite number")
    if not 0.0 <= requested_mm_s <= MOTOR6_GUI_MAX_MM_S:
        raise ValueError(
            f"Motor 6 belt speed must be 0..{MOTOR6_GUI_MAX_MM_S:.2f} mm/s"
        )
    return requested_mm_s * 60.0 / (math.pi * MOTOR6_PULLEY_DIAMETER_MM)


class YuzuOperatorNode(Node):
    def __init__(self) -> None:
        super().__init__("yuzu_operator_gui")
        self.axis1_publisher = self.create_publisher(Float64MultiArray, AXIS1_TOPIC, 10)
        self.axis2_publisher = self.create_publisher(Float64MultiArray, AXIS2_TOPIC, 10)
        self.motor3_publisher = self.create_publisher(Float64, MOTOR3_TOPIC, 10)
        self.motor4_publisher = self.create_publisher(Float64, MOTOR4_TOPIC, 10)
        self.motor5_publisher = self.create_publisher(Float64, MOTOR5_TOPIC, 10)
        self.motor5_zero_publisher = self.create_publisher(Empty, MOTOR5_SET_ZERO_TOPIC, 10)
        self.motor6_publisher = self.create_publisher(Float64MultiArray, MOTOR6_TOPIC, 10)
        self.create_subscription(JointState, "/joint_states", self._joint_state, 10)
        self.create_subscription(
            DynamicJointState, "/dynamic_joint_states", self._dynamic_joint_state, 10
        )
        self.axis1_position_m = None
        self.axis2_velocity_rad_s = None
        self.motor3_position_m = None
        self.motor4_position_m = None
        self.motor5_position_rad = None
        self.motor6_velocity_rad_s = None
        self.feedback_time = None

    def _joint_state(self, message: JointState) -> None:
        if "motor1_motor2" in message.name:
            index = message.name.index("motor1_motor2")
            if index < len(message.position) and math.isfinite(message.position[index]):
                self.axis1_position_m = message.position[index]
            if index < len(message.velocity) and math.isfinite(message.velocity[index]):
                self.axis2_velocity_rad_s = message.velocity[index]
        if "axis1_joint" in message.name:
            index = message.name.index("axis1_joint")
            if index < len(message.position) and math.isfinite(message.position[index]):
                self.axis1_position_m = message.position[index]
        if "axis2_joint" in message.name:
            index = message.name.index("axis2_joint")
            if index < len(message.velocity) and math.isfinite(message.velocity[index]):
                self.axis2_velocity_rad_s = message.velocity[index]
        if "motor4_joint" in message.name:
            index = message.name.index("motor4_joint")
            if index < len(message.position) and math.isfinite(message.position[index]):
                self.motor4_position_m = message.position[index]
            if index < len(message.velocity) and math.isfinite(message.velocity[index]):
                self.motor6_velocity_rad_s = message.velocity[index]
        if "motor6_joint" in message.name:
            index = message.name.index("motor6_joint")
            if index < len(message.velocity) and math.isfinite(message.velocity[index]):
                self.motor6_velocity_rad_s = message.velocity[index]
        self.feedback_time = self.get_clock().now()

    def _dynamic_joint_state(self, message: DynamicJointState) -> None:
        try:
            joint_index = message.joint_names.index("motor1_motor2")
            interfaces = message.interface_values[joint_index]
            interface_index = interfaces.interface_names.index("motor3_position")
            position = interfaces.values[interface_index]
        except (ValueError, IndexError):
            return
        if math.isfinite(position):
            self.motor3_position_m = position
            self.feedback_time = self.get_clock().now()
        try:
            joint_index = message.joint_names.index("motor4_joint")
            interfaces = message.interface_values[joint_index]
            interface_index = interfaces.interface_names.index("motor5_position")
            position = interfaces.values[interface_index]
        except (ValueError, IndexError):
            return
        if math.isfinite(position):
            self.motor5_position_rad = position
            self.feedback_time = self.get_clock().now()

    def publish_axis1(self, position_m: float) -> None:
        message = Float64MultiArray()
        message.data = [position_m]
        self.axis1_publisher.publish(message)

    def publish_axis2(self, rpm: float) -> None:
        message = Float64MultiArray()
        message.data = [rpm]
        self.axis2_publisher.publish(message)

    def publish_motor3(self, position_mm: float) -> None:
        message = Float64()
        message.data = position_mm
        self.motor3_publisher.publish(message)

    def publish_motor4(self, position_mm: float) -> None:
        message = Float64()
        message.data = position_mm
        self.motor4_publisher.publish(message)

    def publish_motor5(self, angle_deg: float) -> None:
        message = Float64()
        message.data = angle_deg
        self.motor5_publisher.publish(message)

    def set_motor5_zero(self) -> None:
        self.motor5_zero_publisher.publish(Empty())

    def publish_motor6(self, rpm: float) -> None:
        message = Float64MultiArray()
        message.data = [rpm]
        self.motor6_publisher.publish(message)

    def feedback_is_fresh(self) -> bool:
        if self.feedback_time is None:
            return False
        age = (self.get_clock().now() - self.feedback_time).nanoseconds / 1e9
        return age <= FEEDBACK_STALE_S


class YuzuOperatorGui:
    def __init__(self, root: tk.Tk, node: YuzuOperatorNode) -> None:
        self.root = root
        self.node = node
        self.axis2_running = False
        self.axis2_command_rpm = 0.0
        self.motor6_running = False
        self.motor6_command_rpm = 0.0
        self.motor5_origin_rad = None

        root.title("Yuzu Peeler — Motor Operator")
        root.geometry("1800x520")
        root.minsize(1500, 490)
        root.protocol("WM_DELETE_WINDOW", self.close)

        style = ttk.Style()
        style.configure("Title.TLabel", font=("Sans", 18, "bold"))
        style.configure("Status.TLabel", font=("Sans", 11, "bold"))
        style.configure("Danger.TButton", font=("Sans", 12, "bold"))

        outer = ttk.Frame(root, padding=16)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="Yuzu Peeler Motor Control", style="Title.TLabel").pack(
            anchor="w"
        )
        ttk.Label(
            outer,
            text="Guarded commissioning controls — the physical power cutoff remains the emergency stop.",
        ).pack(anchor="w", pady=(2, 12))

        panels = ttk.Frame(outer)
        panels.pack(fill="both", expand=True)
        panels.columnconfigure(0, weight=1)
        panels.columnconfigure(1, weight=1)
        panels.columnconfigure(2, weight=1)
        panels.columnconfigure(3, weight=1)
        panels.columnconfigure(4, weight=1)
        panels.columnconfigure(5, weight=1)

        self._build_motor1(panels)
        self._build_motor2(panels)
        self._build_motor3(panels)
        self._build_motor4(panels)
        self._build_motor5(panels)
        self._build_motor6(panels)

        self.details_visible = False
        self.details_button = ttk.Button(
            outer, text="Show motion details ▾", command=self.toggle_motion_details
        )
        self.details_button.pack(anchor="w", pady=(10, 0))
        self.details_frame = self._build_motion_details(outer)

        footer = ttk.Frame(outer)
        footer.pack(fill="x", pady=(12, 0))
        self.connection_text = tk.StringVar(value="Waiting for ROS feedback…")
        ttk.Label(footer, textvariable=self.connection_text, style="Status.TLabel").pack(
            side="left"
        )
        ttk.Button(
            footer,
            text="STOP ROTARY MOTORS",
            style="Danger.TButton",
            command=self.stop_rotary_motors,
        ).pack(side="right")

        self.status_text = tk.StringVar(value="Ready. Start one guarded motor backend.")
        ttk.Label(outer, textvariable=self.status_text, wraplength=640).pack(
            fill="x", pady=(10, 0)
        )
        self.root.after(20, self.tick)

    def _build_motor1(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Motor 1 — Vertical position", padding=12)
        frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        ttk.Label(frame, text="Guarded range: 0.0996–200.0000 mm upward").pack(anchor="w")
        ttk.Label(frame, text="Required speed: 15 mm/s").pack(anchor="w")
        self.axis1_feedback = tk.StringVar(value="Feedback: —")
        ttk.Label(frame, textvariable=self.axis1_feedback, style="Status.TLabel").pack(
            anchor="w", pady=(10, 12)
        )
        row = ttk.Frame(frame)
        row.pack(fill="x")
        ttk.Label(row, text="Target (mm)").pack(side="left")
        self.axis1_entry = ttk.Entry(row, width=12)
        self.axis1_entry.insert(0, "1.9992")
        self.axis1_entry.pack(side="right")
        ttk.Button(row, text=">", width=3, command=lambda: self.step_axis1(1)).pack(side="right", padx=(2, 0))
        ttk.Button(row, text="<", width=3, command=lambda: self.step_axis1(-1)).pack(side="right")
        step_row = ttk.Frame(frame); step_row.pack(fill="x", pady=(4, 0))
        ttk.Label(step_row, text="Step (mm)").pack(side="left")
        self.axis1_step_entry = ttk.Entry(step_row, width=12); self.axis1_step_entry.insert(0, "0.0012"); self.axis1_step_entry.pack(side="right")
        self.axis1_move = ttk.Button(frame, text="Move Motor 1", command=self.move_axis1)
        self.axis1_move.pack(fill="x", pady=(12, 6))
        self.axis1_park = ttk.Button(
            frame, text="Safe park (0.0996 mm)", command=self.park_axis1
        )
        self.axis1_park.pack(fill="x")

    def _build_motor2(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Motor 2 — Yuzu rotation", padding=12)
        frame.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        ttk.Label(frame, text="Validated machine range: up to 250 rpm").pack(anchor="w")
        self.axis2_feedback = tk.StringVar(value="Feedback: —")
        ttk.Label(frame, textvariable=self.axis2_feedback, style="Status.TLabel").pack(
            anchor="w", pady=(10, 12)
        )
        row = ttk.Frame(frame)
        row.pack(fill="x")
        ttk.Label(row, text="Speed (rpm)").pack(side="left")
        self.axis2_entry = ttk.Entry(row, width=12)
        self.axis2_entry.insert(0, "10.0")
        self.axis2_entry.pack(side="right")
        self.axis2_direction = tk.IntVar(value=1)
        directions = ttk.Frame(frame)
        directions.pack(fill="x", pady=(8, 4))
        ttk.Radiobutton(
            directions, text="CW (+)", variable=self.axis2_direction, value=1
        ).pack(side="left")
        ttk.Radiobutton(
            directions, text="CCW (−)", variable=self.axis2_direction, value=-1
        ).pack(side="right")
        self.axis2_start = ttk.Button(frame, text="Start Motor 2", command=self.start_axis2)
        self.axis2_start.pack(fill="x", pady=(8, 6))
        self.axis2_stop = ttk.Button(frame, text="Stop Motor 2", command=self.stop_axis2)
        self.axis2_stop.pack(fill="x")

    def _build_motor4(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Motor 4 — Peeling depth", padding=12)
        frame.grid(row=0, column=3, sticky="nsew", padx=(6, 0))
        ttk.Label(frame, text="Guarded range: 0–15 mm absolute").pack(anchor="w")
        ttk.Label(frame, text="Required speed: 8 mm/s").pack(anchor="w")
        self.motor4_feedback = tk.StringVar(value="Feedback: —")
        ttk.Label(frame, textvariable=self.motor4_feedback, style="Status.TLabel").pack(
            anchor="w", pady=(10, 12)
        )
        row = ttk.Frame(frame)
        row.pack(fill="x")
        ttk.Label(row, text="Target (mm)").pack(side="left")
        self.motor4_entry = ttk.Entry(row, width=12)
        self.motor4_entry.insert(0, "0.500")
        self.motor4_entry.pack(side="right")
        ttk.Button(row, text=">", width=3, command=lambda: self.step_motor4(1)).pack(side="right", padx=(2, 0))
        ttk.Button(row, text="<", width=3, command=lambda: self.step_motor4(-1)).pack(side="right")
        step_row = ttk.Frame(frame); step_row.pack(fill="x", pady=(4, 0))
        ttk.Label(step_row, text="Step (mm)").pack(side="left")
        self.motor4_step_entry = ttk.Entry(step_row, width=12); self.motor4_step_entry.insert(0, "0.001"); self.motor4_step_entry.pack(side="right")
        self.motor4_move = ttk.Button(frame, text="Move Motor 4", command=self.move_motor4)
        self.motor4_move.pack(fill="x", pady=(12, 6))
        self.motor4_home = ttk.Button(frame, text="Return to 0 mm", command=self.home_motor4)
        self.motor4_home.pack(fill="x")

    def _build_motor6(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Motor 6 — Conveyor", padding=12)
        frame.grid(row=0, column=5, sticky="nsew", padx=(6, 0))
        ttk.Label(frame, text="Guarded belt speed: up to 78.54 mm/s").pack(anchor="w")
        ttk.Label(frame, text="+ direction: verified forward").pack(anchor="w")
        self.motor6_feedback = tk.StringVar(value="Feedback: —")
        ttk.Label(frame, textvariable=self.motor6_feedback, style="Status.TLabel").pack(
            anchor="w", pady=(10, 12)
        )
        row = ttk.Frame(frame)
        row.pack(fill="x")
        ttk.Label(row, text="Belt speed (mm/s)").pack(side="left")
        self.motor6_entry = ttk.Entry(row, width=12)
        self.motor6_entry.insert(0, "10.0")
        self.motor6_entry.pack(side="right")
        self.motor6_direction = tk.IntVar(value=1)
        directions = ttk.Frame(frame)
        directions.pack(fill="x", pady=(8, 4))
        ttk.Radiobutton(
            directions, text="Forward (+)", variable=self.motor6_direction, value=1
        ).pack(side="left")
        ttk.Radiobutton(
            directions, text="Reverse (−)", variable=self.motor6_direction, value=-1
        ).pack(side="right")
        self.motor6_start = ttk.Button(
            frame, text="Start conveyor", command=self.start_motor6
        )
        self.motor6_start.pack(fill="x", pady=(8, 6))
        self.motor6_stop = ttk.Button(
            frame, text="Stop conveyor", command=self.stop_motor6
        )
        self.motor6_stop.pack(fill="x")

    def _build_motor5(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Motor 5 — Peeler index", padding=12)
        frame.grid(row=0, column=4, sticky="nsew", padx=(6, 0))
        ttk.Label(frame, text="Set zero before commanding ±90°").pack(anchor="w")
        ttk.Label(frame, text="Guarded speed: 20 rpm").pack(anchor="w")
        self.motor5_feedback = tk.StringVar(value="Feedback: —")
        ttk.Label(frame, textvariable=self.motor5_feedback, style="Status.TLabel").pack(
            anchor="w", pady=(10, 8)
        )
        self.motor5_set_zero = ttk.Button(
            frame, text="Set current position as zero", command=self.set_motor5_zero
        )
        self.motor5_set_zero.pack(fill="x", pady=(0, 8))
        row = ttk.Frame(frame)
        row.pack(fill="x")
        ttk.Label(row, text="Angle (deg)").pack(side="left")
        self.motor5_entry = ttk.Entry(row, width=10)
        self.motor5_entry.insert(0, "90.0")
        self.motor5_entry.pack(side="right")
        ttk.Button(row, text=">", width=3, command=lambda: self.step_motor5(1)).pack(side="right", padx=(2, 0))
        ttk.Button(row, text="<", width=3, command=lambda: self.step_motor5(-1)).pack(side="right")
        step_row = ttk.Frame(frame); step_row.pack(fill="x", pady=(4, 0))
        ttk.Label(step_row, text="Step (deg)").pack(side="left")
        self.motor5_step_entry = ttk.Entry(step_row, width=10); self.motor5_step_entry.insert(0, "1.0"); self.motor5_step_entry.pack(side="right")
        self.motor5_direction = tk.IntVar(value=1)
        directions = ttk.Frame(frame)
        directions.pack(fill="x", pady=(8, 4))
        ttk.Radiobutton(
            directions, text="CW (+)", variable=self.motor5_direction, value=1
        ).pack(side="left")
        ttk.Radiobutton(
            directions, text="CCW (−)", variable=self.motor5_direction, value=-1
        ).pack(side="right")
        self.motor5_move = ttk.Button(frame, text="Move Motor 5", command=self.move_motor5)
        self.motor5_move.pack(fill="x", pady=(8, 6))
        self.motor5_home = ttk.Button(
            frame, text="Return to origin", command=self.home_motor5
        )
        self.motor5_home.pack(fill="x")

    def _build_motor3(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Motor 3 — Peeling depth", padding=12)
        frame.grid(row=0, column=2, sticky="nsew", padx=(6, 0))
        ttk.Label(frame, text="Guarded range: 0–15 mm absolute").pack(anchor="w")
        ttk.Label(frame, text="Required speed: 8 mm/s").pack(anchor="w")
        self.motor3_feedback = tk.StringVar(value="Feedback: —")
        ttk.Label(frame, textvariable=self.motor3_feedback, style="Status.TLabel").pack(
            anchor="w", pady=(10, 12)
        )
        row = ttk.Frame(frame)
        row.pack(fill="x")
        ttk.Label(row, text="Target (mm)").pack(side="left")
        self.motor3_entry = ttk.Entry(row, width=12)
        self.motor3_entry.insert(0, "0.500")
        self.motor3_entry.pack(side="right")
        ttk.Button(row, text=">", width=3, command=lambda: self.step_motor3(1)).pack(side="right", padx=(2, 0))
        ttk.Button(row, text="<", width=3, command=lambda: self.step_motor3(-1)).pack(side="right")
        step_row = ttk.Frame(frame); step_row.pack(fill="x", pady=(4, 0))
        ttk.Label(step_row, text="Step (mm)").pack(side="left")
        self.motor3_step_entry = ttk.Entry(step_row, width=12); self.motor3_step_entry.insert(0, "0.001"); self.motor3_step_entry.pack(side="right")
        self.motor3_move = ttk.Button(frame, text="Move Motor 3", command=self.move_motor3)
        self.motor3_move.pack(fill="x", pady=(12, 6))
        self.motor3_home = ttk.Button(frame, text="Return to 0 mm", command=self.home_motor3)
        self.motor3_home.pack(fill="x")

    def _build_motion_details(self, parent: ttk.Frame) -> ttk.LabelFrame:
        frame = ttk.LabelFrame(parent, text="Configured motion limits", padding=10)
        headings = ("Motor", "Range / maximum", "Velocity", "Acceleration", "Deceleration")
        rows = (
            (
                "Motor 1",
                f"{AXIS1_MIN_MM:.4f}–{AXIS1_MAX_MM:.0f} mm",
                f"{AXIS1_VELOCITY_MM_S:.0f} mm/s",
                f"{AXIS1_ACCELERATION_MM_S2:.0f} mm/s²",
                f"{AXIS1_ACCELERATION_MM_S2:.0f} mm/s²",
            ),
            (
                "Motor 2",
                f"±{AXIS2_GUI_MAX_RPM:.0f} rpm",
                f"{AXIS2_GUI_MAX_RPM:.0f} rpm max",
                f"{AXIS2_ACCELERATION_RPM_S:.0f} rpm/s",
                f"{AXIS2_ACCELERATION_RPM_S:.0f} rpm/s",
            ),
            (
                "Motor 3",
                f"{MOTOR3_MIN_MM:.0f}–{MOTOR3_MAX_MM:.0f} mm",
                f"{MOTOR3_4_VELOCITY_MM_S:.0f} mm/s",
                f"{MOTOR3_ACCELERATION_MM_S2:.0f} mm/s²",
                f"{MOTOR3_ACCELERATION_MM_S2:.0f} mm/s²",
            ),
            (
                "Motor 4",
                f"{MOTOR4_MIN_MM:.0f}–{MOTOR4_MAX_MM:.0f} mm",
                f"{MOTOR3_4_VELOCITY_MM_S:.0f} mm/s",
                f"{MOTOR4_ACCELERATION_MM_S2:.0f} mm/s²",
                f"{MOTOR4_ACCELERATION_MM_S2:.0f} mm/s²",
            ),
            (
                "Motor 6",
                f"±{MOTOR6_GUI_MAX_MM_S:.2f} mm/s",
                f"{MOTOR6_GUI_MAX_MM_S:.2f} mm/s max ({MOTOR6_GUI_MAX_RPM:.0f} rpm)",
                f"{MOTOR6_ACCELERATION_RPM_S * math.pi * MOTOR6_PULLEY_DIAMETER_MM / 60.0:.2f} mm/s²",
                f"{MOTOR6_ACCELERATION_RPM_S * math.pi * MOTOR6_PULLEY_DIAMETER_MM / 60.0:.2f} mm/s²",
            ),
            (
                "Motor 5",
                f"±{MOTOR5_MAX_ANGLE_DEG:.0f}° from runtime zero",
                f"{MOTOR5_VELOCITY_RPM:.0f} rpm",
                f"{MOTOR5_ACCELERATION_RPM_S:.0f} rpm/s",
                f"{MOTOR5_ACCELERATION_RPM_S:.0f} rpm/s",
            ),
        )
        for column, heading in enumerate(headings):
            frame.columnconfigure(column, weight=1)
            ttk.Label(frame, text=heading, style="Status.TLabel").grid(
                row=0, column=column, sticky="w", padx=(0, 18), pady=(0, 5)
            )
        for row_index, row in enumerate(rows, start=1):
            for column, value in enumerate(row):
                ttk.Label(frame, text=value).grid(
                    row=row_index, column=column, sticky="w", padx=(0, 18), pady=2
                )
        ttk.Label(
            frame,
            text=(
                "Acceleration and deceleration are symmetric software ramps."
            ),
        ).grid(row=len(rows) + 1, column=0, columnspan=5, sticky="w", pady=(7, 0))
        return frame

    def toggle_motion_details(self) -> None:
        self.details_visible = not self.details_visible
        if self.details_visible:
            self.details_frame.pack(fill="x", pady=(6, 0), before=self.details_button.master.winfo_children()[-2])
            self.details_button.configure(text="Hide motion details ▴")
        else:
            self.details_frame.pack_forget()
            self.details_button.configure(text="Show motion details ▾")

    def step_target(self, entry, step_entry, direction, minimum, maximum, callback) -> None:
        try:
            value = float(entry.get()) + direction * abs(float(step_entry.get()))
        except ValueError:
            messagebox.showerror("Invalid step", "Enter a finite positive step size.")
            return
        value = max(minimum, min(maximum, value))
        entry.delete(0, tk.END)
        entry.insert(0, f"{value:.4f}" if maximum >= 100.0 else f"{value:.3f}")
        callback()

    def step_axis1(self, direction):
        self.step_target(self.axis1_entry, self.axis1_step_entry, direction, AXIS1_MIN_MM, AXIS1_MAX_MM, self.move_axis1)

    def step_motor3(self, direction):
        self.step_target(self.motor3_entry, self.motor3_step_entry, direction, MOTOR3_MIN_MM, MOTOR3_MAX_MM, self.move_motor3)

    def step_motor4(self, direction):
        self.step_target(self.motor4_entry, self.motor4_step_entry, direction, MOTOR4_MIN_MM, MOTOR4_MAX_MM, self.move_motor4)

    def step_motor5(self, direction):
        self.step_target(self.motor5_entry, self.motor5_step_entry, direction, 0.0, MOTOR5_MAX_ANGLE_DEG, self.move_motor5)

    def move_axis1(self) -> None:
        if self.node.axis1_publisher.get_subscription_count() == 0:
            messagebox.showerror("Motor 1 unavailable", "Axis 1 guarded backend is not running.")
            return
        try:
            requested_mm = float(self.axis1_entry.get())
            position_m = aligned_axis1_position_m(requested_mm)
        except ValueError as error:
            messagebox.showerror("Invalid Motor 1 command", str(error))
            return
        self.node.publish_axis1(position_m)
        self.status_text.set(f"Motor 1 command accepted by GUI: {position_m * 1000:.4f} mm")

    def park_axis1(self) -> None:
        self.axis1_entry.delete(0, tk.END)
        self.axis1_entry.insert(0, f"{AXIS1_PARK_MM:.4f}")
        self.move_axis1()

    def start_axis2(self) -> None:
        if self.node.axis2_publisher.get_subscription_count() == 0:
            messagebox.showerror("Motor 2 unavailable", "Axis 2 guarded backend is not running.")
            return
        try:
            rpm = checked_axis2_rpm(float(self.axis2_entry.get()))
        except ValueError as error:
            messagebox.showerror("Invalid Motor 2 command", str(error))
            return
        self.axis2_command_rpm = rpm * self.axis2_direction.get()
        self.axis2_running = rpm > 0.0
        self.node.publish_axis2(self.axis2_command_rpm)
        self.status_text.set(f"Motor 2 command active: {self.axis2_command_rpm:.1f} rpm")

    def stop_axis2(self) -> None:
        self.axis2_running = False
        self.axis2_command_rpm = 0.0
        self.node.publish_axis2(0.0)
        self.status_text.set("Motor 2 stop requested; guarded deceleration is active.")

    def move_motor3(self) -> None:
        if self.node.motor3_publisher.get_subscription_count() == 0:
            messagebox.showerror("Motor 3 unavailable", "Motor 3 guarded backend is not running.")
            return
        try:
            target_mm = aligned_motor3_position_mm(float(self.motor3_entry.get()))
        except ValueError as error:
            messagebox.showerror("Invalid Motor 3 command", str(error))
            return
        self.node.publish_motor3(target_mm)
        self.status_text.set(f"Motor 3 command accepted by GUI: {target_mm:.3f} mm")

    def home_motor3(self) -> None:
        self.motor3_entry.delete(0, tk.END)
        self.motor3_entry.insert(0, "0.000")
        self.move_motor3()

    def move_motor4(self) -> None:
        if self.node.motor4_publisher.get_subscription_count() == 0:
            messagebox.showerror("Motor 4 unavailable", "Motor 4 guarded backend is not running.")
            return
        try:
            target_mm = aligned_motor4_position_mm(float(self.motor4_entry.get()))
        except ValueError as error:
            messagebox.showerror("Invalid Motor 4 command", str(error))
            return
        self.node.publish_motor4(target_mm)
        self.status_text.set(f"Motor 4 command accepted by GUI: {target_mm:.3f} mm")

    def home_motor4(self) -> None:
        self.motor4_entry.delete(0, tk.END)
        self.motor4_entry.insert(0, "0.000")
        self.move_motor4()

    def set_motor5_zero(self) -> None:
        if self.node.motor5_zero_publisher.get_subscription_count() == 0:
            messagebox.showerror("Motor 5 unavailable", "Motor 5 guarded backend is not running.")
            return
        if self.node.motor5_position_rad is None:
            messagebox.showerror("Motor 5 unavailable", "No valid Motor 5 feedback is available.")
            return
        self.node.set_motor5_zero()
        self.motor5_origin_rad = self.node.motor5_position_rad
        self.status_text.set("Motor 5 runtime zero captured at the current mechanism pose.")

    def move_motor5(self) -> None:
        if self.node.motor5_publisher.get_subscription_count() == 0:
            messagebox.showerror("Motor 5 unavailable", "Motor 5 guarded backend is not running.")
            return
        if self.motor5_origin_rad is None:
            messagebox.showerror("Motor 5 zero required", "Set the current Motor 5 position as zero first.")
            return
        try:
            angle_deg = float(self.motor5_entry.get())
            if not math.isfinite(angle_deg) or not 0.0 <= angle_deg <= MOTOR5_MAX_ANGLE_DEG:
                raise ValueError("Motor 5 angle must be 0..90 degrees")
        except ValueError as error:
            messagebox.showerror("Invalid Motor 5 command", str(error))
            return
        signed_angle_deg = angle_deg * self.motor5_direction.get()
        self.node.publish_motor5(signed_angle_deg)
        self.status_text.set(f"Motor 5 command accepted by GUI: {signed_angle_deg:+.1f}°")

    def home_motor5(self) -> None:
        if self.motor5_origin_rad is None:
            messagebox.showerror("Motor 5 zero required", "Set the current Motor 5 position as zero first.")
            return
        self.node.publish_motor5(0.0)
        self.status_text.set("Motor 5 return-to-origin requested.")

    def start_motor6(self) -> None:
        if self.node.motor6_publisher.get_subscription_count() == 0:
            messagebox.showerror("Motor 6 unavailable", "Motor 6 guarded backend is not running.")
            return
        try:
            rpm = checked_motor6_speed_mm_s(float(self.motor6_entry.get()))
        except ValueError as error:
            messagebox.showerror("Invalid Motor 6 command", str(error))
            return
        self.motor6_command_rpm = rpm * self.motor6_direction.get()
        self.motor6_running = rpm > 0.0
        self.node.publish_motor6(self.motor6_command_rpm)
        belt_speed_mm_s = (
            abs(self.motor6_command_rpm) * math.pi * MOTOR6_PULLEY_DIAMETER_MM / 60.0
        )
        self.status_text.set(
            f"Motor 6 conveyor command active: {belt_speed_mm_s:.2f} mm/s "
            f"({self.motor6_command_rpm:.2f} rpm)"
        )

    def stop_motor6(self) -> None:
        self.motor6_running = False
        self.motor6_command_rpm = 0.0
        self.node.publish_motor6(0.0)
        self.status_text.set("Motor 6 stop requested; guarded deceleration is active.")

    def stop_rotary_motors(self) -> None:
        self.axis2_running = False
        self.axis2_command_rpm = 0.0
        self.motor6_running = False
        self.motor6_command_rpm = 0.0
        for _ in range(5):
            self.node.publish_axis2(0.0)
            self.node.publish_motor6(0.0)
        self.status_text.set("Motor 2 and Motor 6 guarded stops requested.")

    def tick(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)
        axis1_ready = self.node.axis1_publisher.get_subscription_count() > 0
        axis2_ready = self.node.axis2_publisher.get_subscription_count() > 0
        motor3_ready = self.node.motor3_publisher.get_subscription_count() > 0
        motor4_ready = self.node.motor4_publisher.get_subscription_count() > 0
        motor5_ready = (
            self.node.motor5_publisher.get_subscription_count() > 0
            and self.node.motor5_zero_publisher.get_subscription_count() > 0
        )
        motor6_ready = self.node.motor6_publisher.get_subscription_count() > 0
        self.axis1_move.configure(state="normal" if axis1_ready else "disabled")
        self.axis1_park.configure(state="normal" if axis1_ready else "disabled")
        self.axis2_start.configure(state="normal" if axis2_ready else "disabled")
        self.axis2_stop.configure(state="normal" if axis2_ready else "disabled")
        self.motor3_move.configure(state="normal" if motor3_ready else "disabled")
        self.motor3_home.configure(state="normal" if motor3_ready else "disabled")
        self.motor4_move.configure(state="normal" if motor4_ready else "disabled")
        self.motor4_home.configure(state="normal" if motor4_ready else "disabled")
        self.motor5_set_zero.configure(state="normal" if motor5_ready else "disabled")
        motor5_motion_ready = motor5_ready and self.motor5_origin_rad is not None
        self.motor5_move.configure(state="normal" if motor5_motion_ready else "disabled")
        self.motor5_home.configure(state="normal" if motor5_motion_ready else "disabled")
        self.motor6_start.configure(state="normal" if motor6_ready else "disabled")
        self.motor6_stop.configure(state="normal" if motor6_ready else "disabled")

        if self.axis2_running:
            self.node.publish_axis2(self.axis2_command_rpm)
        if self.motor6_running:
            self.node.publish_motor6(self.motor6_command_rpm)

        if self.node.axis1_position_m is None:
            self.axis1_feedback.set("Feedback: —")
        else:
            self.axis1_feedback.set(
                f"Feedback: {self.node.axis1_position_m * 1000:.4f} mm"
            )
        if self.node.axis2_velocity_rad_s is None:
            self.axis2_feedback.set("Feedback: —")
        else:
            rpm = self.node.axis2_velocity_rad_s * 30.0 / math.pi
            self.axis2_feedback.set(f"Feedback: {rpm:.2f} rpm")
        if self.node.motor4_position_m is None:
            self.motor4_feedback.set("Feedback: —")
        else:
            self.motor4_feedback.set(
                f"Feedback: {self.node.motor4_position_m * 1000:.3f} mm"
            )
        if self.node.motor3_position_m is None:
            self.motor3_feedback.set("Feedback: —")
        else:
            self.motor3_feedback.set(
                f"Feedback: {self.node.motor3_position_m * 1000:.3f} mm"
            )
        if self.node.motor6_velocity_rad_s is None:
            self.motor6_feedback.set("Feedback: —")
        else:
            rpm = self.node.motor6_velocity_rad_s * 30.0 / math.pi
            belt_speed_mm_s = rpm * math.pi * MOTOR6_PULLEY_DIAMETER_MM / 60.0
            self.motor6_feedback.set(
                f"Feedback: {rpm:.2f} rpm ({belt_speed_mm_s:.2f} mm/s)"
            )
        if self.node.motor5_position_rad is None:
            self.motor5_feedback.set("Feedback: —")
        elif self.motor5_origin_rad is None:
            self.motor5_feedback.set("Feedback ready — set runtime zero")
        else:
            relative_deg = math.degrees(
                self.node.motor5_position_rad - self.motor5_origin_rad
            )
            self.motor5_feedback.set(f"Feedback: {relative_deg:+.3f}°")

        backends = []
        if axis1_ready:
            backends.append("Motor 1")
        if axis2_ready:
            backends.append("Motor 2")
        if motor3_ready:
            backends.append("Motor 3")
        if motor4_ready:
            backends.append("Motor 4")
        if motor5_ready:
            backends.append("Motor 5")
        if motor6_ready:
            backends.append("Motor 6")
        freshness = "feedback live" if self.node.feedback_is_fresh() else "feedback stale"
        self.connection_text.set(
            f"Backend: {', '.join(backends) if backends else 'none'} — {freshness}"
        )
        self.root.after(100, self.tick)

    def close(self) -> None:
        self.axis2_running = False
        self.motor6_running = False
        for _ in range(5):
            self.node.publish_axis2(0.0)
            self.node.publish_motor6(0.0)
        self.root.destroy()


def main() -> None:
    rclpy.init()
    node = YuzuOperatorNode()
    root = tk.Tk()
    YuzuOperatorGui(root, node)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
