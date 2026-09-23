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
AXIS1_MIN_MM = 0.1008
AXIS1_MAX_MM = 200.0
AXIS1_INCREMENT_MM = 0.0012
AXIS1_PARK_MM = 0.1008
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
            f"Motor 1 position must be at least {AXIS1_MIN_MM:.3f} mm and no more than {AXIS1_MAX_MM:.1f} mm"
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
        self.feedback_times = {
            "motor1": None,
            "motor2": None,
            "motor3": None,
            "motor4": None,
            "motor5": None,
            "motor6": None,
        }

    def _mark_feedback(self, motor: str) -> None:
        self.feedback_times[motor] = self.get_clock().now()

    def _joint_state(self, message: JointState) -> None:
        if "motor1_motor2" in message.name:
            index = message.name.index("motor1_motor2")
            if index < len(message.position) and math.isfinite(message.position[index]):
                self.axis1_position_m = message.position[index]
                self._mark_feedback("motor1")
            if index < len(message.velocity) and math.isfinite(message.velocity[index]):
                self.axis2_velocity_rad_s = message.velocity[index]
                self._mark_feedback("motor2")
        if "axis1_joint" in message.name:
            index = message.name.index("axis1_joint")
            if index < len(message.position) and math.isfinite(message.position[index]):
                self.axis1_position_m = message.position[index]
                self._mark_feedback("motor1")
        if "axis2_joint" in message.name:
            index = message.name.index("axis2_joint")
            if index < len(message.velocity) and math.isfinite(message.velocity[index]):
                self.axis2_velocity_rad_s = message.velocity[index]
                self._mark_feedback("motor2")
        if "motor4_joint" in message.name:
            index = message.name.index("motor4_joint")
            if index < len(message.position) and math.isfinite(message.position[index]):
                self.motor4_position_m = message.position[index]
                self._mark_feedback("motor4")
            if index < len(message.velocity) and math.isfinite(message.velocity[index]):
                self.motor6_velocity_rad_s = message.velocity[index]
                self._mark_feedback("motor6")
        if "motor6_joint" in message.name:
            index = message.name.index("motor6_joint")
            if index < len(message.velocity) and math.isfinite(message.velocity[index]):
                self.motor6_velocity_rad_s = message.velocity[index]
                self._mark_feedback("motor6")

    def _dynamic_joint_state(self, message: DynamicJointState) -> None:
        channels = (
            ("motor1_motor2", "motor3_position", "motor3_position_m", "motor3"),
            ("motor4_joint", "motor5_position", "motor5_position_rad", "motor5"),
        )
        for joint_name, interface_name, attribute, motor in channels:
            try:
                joint_index = message.joint_names.index(joint_name)
                interfaces = message.interface_values[joint_index]
                interface_index = interfaces.interface_names.index(interface_name)
                position = interfaces.values[interface_index]
            except (ValueError, IndexError):
                continue
            if math.isfinite(position):
                setattr(self, attribute, position)
                self._mark_feedback(motor)

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

    def feedback_age(self, motor: str) -> float | None:
        stamp = self.feedback_times[motor]
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def feedback_is_fresh_for(self, motor: str) -> bool:
        age = self.feedback_age(motor)
        return age is not None and age <= FEEDBACK_STALE_S


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
        root.geometry("1500x900")
        root.minsize(1050, 760)
        root.protocol("WM_DELETE_WINDOW", self.close)

        style = ttk.Style()
        style.configure("Title.TLabel", font=("Sans", 18, "bold"))
        style.configure("Status.TLabel", font=("Sans", 11, "bold"))
        style.configure("Banner.TLabel", font=("Sans", 12, "bold"))

        outer = ttk.Frame(root, padding=16)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="Yuzu Peeler Motor Control", style="Title.TLabel").pack(
            anchor="w"
        )
        ttk.Label(
            outer,
            text="SOFTWARE CONTROL ONLY — physical power cutoff is the emergency stop.",
            style="Banner.TLabel",
        ).pack(anchor="w", fill="x", pady=(4, 8))

        self.machine_banner = tk.Label(
            outer,
            text="BACKEND OFFLINE · waiting for motor feedback",
            anchor="w",
            padx=10,
            pady=8,
            bg="#5b6470",
            fg="white",
            font=("Sans", 12, "bold"),
        )
        self.machine_banner.pack(fill="x", pady=(0, 10))

        safety_row = ttk.Frame(outer)
        safety_row.pack(fill="x", pady=(0, 8))
        self.connection_text = tk.StringVar(value="No motor backends connected")
        ttk.Label(safety_row, textvariable=self.connection_text, style="Status.TLabel").pack(side="left")
        self.rotary_stop_button = tk.Button(
            safety_row,
            text="STOP ROTARY MOTORS · CONTROLLED STOP",
            command=self.stop_rotary_motors,
            bg="#b42318",
            fg="white",
            activebackground="#7a271a",
            activeforeground="white",
            font=("Sans", 12, "bold"),
            padx=12,
            pady=8,
            relief="raised",
        )
        self.rotary_stop_button.pack(side="right")

        panels = ttk.Frame(outer)
        panels.pack(fill="both", expand=True)
        for column in range(3):
            panels.columnconfigure(column, weight=1, uniform="motor_panels")
        panels.rowconfigure(0, weight=1)
        panels.rowconfigure(1, weight=1)

        self.step_buttons = {motor: [] for motor in range(1, 7)}
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

        self.status_text = tk.StringVar(value="Commands are sent to guarded backends; completion is not reported by this GUI.")
        self.command_status_label = ttk.Label(
            outer, textvariable=self.status_text, wraplength=900
        )
        self.command_status_label.pack(
            fill="x", pady=(10, 0)
        )
        self.root.after(20, self.tick)

    def _build_motor1(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Motor 1 — Vertical position", padding=12)
        frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        ttk.Label(frame, text="GUI range: 0.101–200 mm upward").pack(anchor="w")
        ttk.Label(frame, text="Required speed: 15 mm/s").pack(anchor="w")
        ttk.Label(frame, text="Expanded travel range requires staged validation").pack(anchor="w")
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
        for label, direction in ((">", 1), ("<", -1)):
            button = ttk.Button(row, text=label, width=3, command=lambda d=direction: self.step_axis1(d))
            button.pack(side="right", padx=(2, 0) if direction > 0 else (0, 0))
            self.step_buttons[1].append(button)
        step_row = ttk.Frame(frame); step_row.pack(fill="x", pady=(4, 0))
        ttk.Label(step_row, text="Step (mm)").pack(side="left")
        self.axis1_step_entry = ttk.Entry(step_row, width=12); self.axis1_step_entry.insert(0, "0.0012"); self.axis1_step_entry.pack(side="right")
        self.axis1_move = ttk.Button(frame, text="Move Motor 1", command=self.move_axis1)
        self.axis1_move.pack(fill="x", pady=(12, 6))
        self.axis1_park = ttk.Button(
            frame, text="Safe park (0.101 mm)", command=self.park_axis1
        )
        self.axis1_park.pack(fill="x")

    def _build_motor2(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Motor 2 — Yuzu rotation", padding=12)
        frame.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        ttk.Label(frame, text="Validated machine range: up to 250 rpm").pack(anchor="w")
        ttk.Label(frame, text="Combined GUI control physically validated").pack(anchor="w")
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
        frame = ttk.LabelFrame(parent, text="Motor 4 — Peeling depth · slave 1 / Axis 1", padding=12)
        frame.grid(row=0, column=2, sticky="nsew", padx=4, pady=4)
        ttk.Label(frame, text="Guarded range: 0–15 mm absolute").pack(anchor="w")
        ttk.Label(frame, text="Required speed: 8 mm/s").pack(anchor="w")
        ttk.Label(frame, text="Combined GUI motion physically validated").pack(anchor="w")
        self.motor4_feedback = tk.StringVar(value="Feedback: —")
        ttk.Label(frame, textvariable=self.motor4_feedback, style="Status.TLabel").pack(
            anchor="w", pady=(10, 12)
        )
        self.motor4_travel = ttk.Progressbar(frame, maximum=MOTOR4_MAX_MM, mode="determinate")
        self.motor4_travel.pack(fill="x", pady=(0, 8))
        row = ttk.Frame(frame)
        row.pack(fill="x")
        ttk.Label(row, text="Target (mm)").pack(side="left")
        self.motor4_entry = ttk.Entry(row, width=12)
        self.motor4_entry.insert(0, "0.500")
        self.motor4_entry.pack(side="right")
        for label, direction in ((">", 1), ("<", -1)):
            button = ttk.Button(row, text=label, width=3, command=lambda d=direction: self.step_motor4(d))
            button.pack(side="right", padx=(2, 0) if direction > 0 else (0, 0))
            self.step_buttons[4].append(button)
        step_row = ttk.Frame(frame); step_row.pack(fill="x", pady=(4, 0))
        ttk.Label(step_row, text="Step (mm)").pack(side="left")
        self.motor4_step_entry = ttk.Entry(step_row, width=12); self.motor4_step_entry.insert(0, "0.001"); self.motor4_step_entry.pack(side="right")
        self.motor4_move = ttk.Button(frame, text="Move Motor 4", command=self.move_motor4)
        self.motor4_move.pack(fill="x", pady=(12, 6))
        self.motor4_home = ttk.Button(frame, text="Return to 0 mm", command=self.home_motor4)
        self.motor4_home.pack(fill="x")

    def _build_motor6(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Motor 6 — Conveyor", padding=12)
        frame.grid(row=1, column=2, sticky="nsew", padx=4, pady=4)
        ttk.Label(frame, text="Guarded belt speed: up to 78.54 mm/s").pack(anchor="w")
        ttk.Label(frame, text="+ direction: verified forward").pack(anchor="w")
        ttk.Label(frame, text="Combined GUI physical validation required").pack(anchor="w")
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
        frame.grid(row=1, column=1, sticky="nsew", padx=4, pady=4)
        ttk.Label(frame, text="Set zero before commanding ±90°").pack(anchor="w")
        ttk.Label(frame, text="Guarded speed: 20 rpm").pack(anchor="w")
        ttk.Label(frame, text="Combined 90° validation required").pack(anchor="w")
        self.motor5_feedback = tk.StringVar(value="Feedback: —")
        ttk.Label(frame, textvariable=self.motor5_feedback, style="Status.TLabel").pack(
            anchor="w", pady=(10, 8)
        )
        ttk.Label(frame, text="−90° from zero     0°     +90° from zero").pack(anchor="w")
        self.motor5_angle_bar = ttk.Progressbar(frame, maximum=180.0, mode="determinate")
        self.motor5_angle_bar.pack(fill="x", pady=(0, 8))
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
        for label, direction in ((">", 1), ("<", -1)):
            button = ttk.Button(row, text=label, width=3, command=lambda d=direction: self.step_motor5(d))
            button.pack(side="right", padx=(2, 0) if direction > 0 else (0, 0))
            self.step_buttons[5].append(button)
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
        frame = ttk.LabelFrame(parent, text="Motor 3 — Peeling depth · slave 0 / Axis 3", padding=12)
        frame.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)
        ttk.Label(frame, text="Guarded range: 0–15 mm absolute").pack(anchor="w")
        ttk.Label(frame, text="Required speed: 8 mm/s").pack(anchor="w")
        ttk.Label(frame, text="Combined GUI motion physically validated").pack(anchor="w")
        self.motor3_feedback = tk.StringVar(value="Feedback: —")
        ttk.Label(frame, textvariable=self.motor3_feedback, style="Status.TLabel").pack(
            anchor="w", pady=(10, 12)
        )
        self.motor3_travel = ttk.Progressbar(frame, maximum=MOTOR3_MAX_MM, mode="determinate")
        self.motor3_travel.pack(fill="x", pady=(0, 8))
        row = ttk.Frame(frame)
        row.pack(fill="x")
        ttk.Label(row, text="Target (mm)").pack(side="left")
        self.motor3_entry = ttk.Entry(row, width=12)
        self.motor3_entry.insert(0, "0.500")
        self.motor3_entry.pack(side="right")
        for label, direction in ((">", 1), ("<", -1)):
            button = ttk.Button(row, text=label, width=3, command=lambda d=direction: self.step_motor3(d))
            button.pack(side="right", padx=(2, 0) if direction > 0 else (0, 0))
            self.step_buttons[3].append(button)
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
                f"{AXIS1_MIN_MM:.3f}–{AXIS1_MAX_MM:.0f} mm",
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
                f"{MOTOR6_GUI_MAX_MM_S:.2f} mm/s max",
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
            self.details_frame.pack(fill="x", pady=(6, 0), before=self.command_status_label)
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
        self.status_text.set(f"Motor 1 command sent: {position_m * 1000:.4f} mm · monitor feedback for completion")

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
        self.status_text.set(f"Motor 2 speed command sent: {self.axis2_command_rpm:.1f} rpm")

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
        self.status_text.set(f"Motor 3 command sent: {target_mm:.3f} mm · monitor feedback for completion")

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
        self.status_text.set(f"Motor 4 command sent: {target_mm:.3f} mm · monitor feedback for completion")

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
        if not self.node.feedback_is_fresh_for("motor5"):
            messagebox.showerror("Motor 5 feedback stale", "Wait for fresh Motor 5 feedback before capturing zero.")
            return
        if not messagebox.askyesno(
            "Confirm Motor 5 stopped",
            "Confirm the mechanism is stationary and at the intended zero pose. "
            "The GUI has no independent drive-state or velocity interlock for this action.",
        ):
            return
        self.node.set_motor5_zero()
        self.motor5_origin_rad = self.node.motor5_position_rad
        self.status_text.set("Motor 5 zero request sent; verify the displayed relative position before moving.")

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
        self.status_text.set(f"Motor 5 command sent: {signed_angle_deg:+.1f}° · monitor feedback for completion")

    def home_motor5(self) -> None:
        if self.motor5_origin_rad is None:
            messagebox.showerror("Motor 5 zero required", "Set the current Motor 5 position as zero first.")
            return
        self.node.publish_motor5(0.0)
        self.status_text.set("Motor 5 return-to-origin command sent; monitor feedback for completion.")

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
        self.status_text.set(f"Motor 6 conveyor command sent: {belt_speed_mm_s:.2f} mm/s")

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
        backend = {
            "motor1": self.node.axis1_publisher.get_subscription_count() > 0,
            "motor2": self.node.axis2_publisher.get_subscription_count() > 0,
            "motor3": self.node.motor3_publisher.get_subscription_count() > 0,
            "motor4": self.node.motor4_publisher.get_subscription_count() > 0,
            "motor5": self.node.motor5_publisher.get_subscription_count() > 0
            and self.node.motor5_zero_publisher.get_subscription_count() > 0,
            "motor6": self.node.motor6_publisher.get_subscription_count() > 0,
        }
        fresh = {
            motor: self.node.feedback_is_fresh_for(motor) for motor in backend
        }
        ready = {motor: backend[motor] and fresh[motor] for motor in backend}

        if self.axis2_running and not ready["motor2"]:
            self.axis2_running = False
            self.axis2_command_rpm = 0.0
            for _ in range(5):
                self.node.publish_axis2(0.0)
            self.status_text.set("Motor 2 feedback lost; repeated controlled stop commands sent.")
        if self.motor6_running and not ready["motor6"]:
            self.motor6_running = False
            self.motor6_command_rpm = 0.0
            for _ in range(5):
                self.node.publish_motor6(0.0)
            self.status_text.set("Motor 6 feedback lost; repeated controlled stop commands sent.")

        axis1_ready = ready["motor1"]
        axis2_ready = ready["motor2"]
        motor3_ready = ready["motor3"]
        motor4_ready = ready["motor4"]
        motor5_ready = backend["motor5"] and ready["motor5"]
        motor6_ready = ready["motor6"]
        self.axis1_move.configure(state="normal" if axis1_ready else "disabled")
        self.axis1_park.configure(state="normal" if axis1_ready else "disabled")
        self.axis2_start.configure(state="normal" if axis2_ready else "disabled")
        self.axis2_stop.configure(state="normal" if backend["motor2"] else "disabled")
        self.motor3_move.configure(state="normal" if motor3_ready else "disabled")
        self.motor3_home.configure(state="normal" if motor3_ready else "disabled")
        self.motor4_move.configure(state="normal" if motor4_ready else "disabled")
        self.motor4_home.configure(state="normal" if motor4_ready else "disabled")
        self.motor5_set_zero.configure(state="normal" if motor5_ready else "disabled")
        motor5_motion_ready = motor5_ready and self.motor5_origin_rad is not None
        self.motor5_move.configure(state="normal" if motor5_motion_ready else "disabled")
        self.motor5_home.configure(state="normal" if motor5_motion_ready else "disabled")
        self.motor6_start.configure(state="normal" if motor6_ready else "disabled")
        self.motor6_stop.configure(state="normal" if backend["motor6"] else "disabled")
        for motor_id in (1, 3, 4):
            state = "normal" if ready[f"motor{motor_id}"] else "disabled"
            for button in self.step_buttons[motor_id]:
                button.configure(state=state)
        for button in self.step_buttons[5]:
            button.configure(state="normal" if motor5_motion_ready else "disabled")

        if self.axis2_running:
            self.node.publish_axis2(self.axis2_command_rpm)
        if self.motor6_running:
            self.node.publish_motor6(self.motor6_command_rpm)

        if self.node.axis1_position_m is None:
            self.axis1_feedback.set("Waiting for Motor 1 feedback")
        else:
            state = "FEEDBACK LIVE" if fresh["motor1"] else "FEEDBACK STALE"
            self.axis1_feedback.set(
                f"{state} · Actual: {self.node.axis1_position_m * 1000:.3f} mm"
            )
        if self.node.axis2_velocity_rad_s is None:
            self.axis2_feedback.set("Waiting for Motor 2 feedback")
        else:
            rpm = self.node.axis2_velocity_rad_s * 30.0 / math.pi
            state = "FEEDBACK LIVE" if fresh["motor2"] else "FEEDBACK STALE"
            self.axis2_feedback.set(f"{state} · Actual: {rpm:.2f} rpm")
        if self.node.motor4_position_m is None:
            self.motor4_feedback.set("Waiting for Motor 4 feedback")
            self.motor4_travel.configure(value=0.0)
        else:
            state = "FEEDBACK LIVE" if fresh["motor4"] else "FEEDBACK STALE"
            motor4_mm = self.node.motor4_position_m * 1000.0
            self.motor4_feedback.set(
                f"{state} · Actual: {motor4_mm:.3f} mm / 15 mm"
            )
            self.motor4_travel.configure(value=max(0.0, min(MOTOR4_MAX_MM, motor4_mm)))
        if self.node.motor3_position_m is None:
            self.motor3_feedback.set("Waiting for Motor 3 feedback")
            self.motor3_travel.configure(value=0.0)
        else:
            state = "FEEDBACK LIVE" if fresh["motor3"] else "FEEDBACK STALE"
            motor3_mm = self.node.motor3_position_m * 1000.0
            self.motor3_feedback.set(
                f"{state} · Actual: {motor3_mm:.3f} mm / 15 mm"
            )
            self.motor3_travel.configure(value=max(0.0, min(MOTOR3_MAX_MM, motor3_mm)))
        if self.node.motor6_velocity_rad_s is None:
            self.motor6_feedback.set("Waiting for Motor 6 feedback")
        else:
            rpm = self.node.motor6_velocity_rad_s * 30.0 / math.pi
            belt_speed_mm_s = rpm * math.pi * MOTOR6_PULLEY_DIAMETER_MM / 60.0
            state = "FEEDBACK LIVE" if fresh["motor6"] else "FEEDBACK STALE"
            self.motor6_feedback.set(f"{state} · Actual: {belt_speed_mm_s:.2f} mm/s")
        if self.node.motor5_position_rad is None:
            self.motor5_feedback.set("Waiting for Motor 5 feedback")
        elif self.motor5_origin_rad is None:
            state = "FEEDBACK LIVE" if fresh["motor5"] else "FEEDBACK STALE"
            self.motor5_feedback.set(f"{state} · Set runtime zero after confirming stopped")
            self.motor5_angle_bar.configure(value=90.0)
        else:
            relative_deg = math.degrees(
                self.node.motor5_position_rad - self.motor5_origin_rad
            )
            state = "FEEDBACK LIVE" if fresh["motor5"] else "FEEDBACK STALE"
            self.motor5_feedback.set(f"{state} · Relative: {relative_deg:+.3f}°")
            self.motor5_angle_bar.configure(value=max(0.0, min(180.0, relative_deg + 90.0)))

        fresh_count = sum(ready.values())
        connected_count = sum(backend.values())
        self.connection_text.set(
            f"Guard topic connections: {connected_count}/6 · fresh axis feedback: {fresh_count}/6"
        )
        if connected_count == 0:
            banner, color = "BACKEND OFFLINE · motor commands disabled", "#5b6470"
        elif fresh_count != connected_count:
            banner, color = "FEEDBACK MISSING OR STALE · check each motor panel", "#b54708"
        else:
            banner, color = (
                "FEEDBACK FRESH · drive readiness and alarms are not reported by this GUI",
                "#175cd3",
            )
        self.machine_banner.configure(text=banner, bg=color)
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
