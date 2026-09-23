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
MOTOR5_MANUAL_STEP_TOPIC = "/motor5_position_controller/manual_step_deg"
MOTOR6_TOPIC = "/motor6_conveyor/commands_rpm"
AXIS1_MIN_MM = 0.0
AXIS1_MAX_MM = 400.0
AXIS1_INCREMENT_MM = 0.0012
AXIS1_PARK_MM = 0.0
AXIS1_VELOCITY_MM_S = 15.0
AXIS1_ACCELERATION_MM_S2 = 15.0
AXIS2_GUI_MAX_RPM = 250.0  # project operating limit; drive maximum is 416 rpm
AXIS2_ACCELERATION_RPM_S = 500.0
MOTOR4_MIN_MM = 0.0
MOTOR4_MAX_MM = 20.0
MOTOR4_INCREMENT_MM = 0.0001
MOTOR3_MIN_MM = 0.0
MOTOR3_MAX_MM = 20.0
MOTOR3_INCREMENT_MM = 0.0001
MOTOR3_4_VELOCITY_MM_S = 8.0
MOTOR3_ACCELERATION_MM_S2 = 50.0
MOTOR4_ACCELERATION_MM_S2 = 50.0
MOTOR5_MAX_ANGLE_DEG = 180.0
MOTOR5_HARDWARE_STEP_DEG = 0.0018
MOTOR5_VELOCITY_RPM = 20.0
MOTOR5_ACCELERATION_RPM_S = 20.0
MOTOR6_GUI_MAX_RPM = 60.0
MOTOR6_ACCELERATION_RPM_S = 250.0
MOTOR6_PULLEY_DIAMETER_MM = 30.0
MOTOR6_GUI_MAX_MM_S = 90.0
MOTOR6_STEP_MAX_MM = 1000.0
FEEDBACK_STALE_S = 2.0
OPERATION_ENABLED_MASK = 0x006F
OPERATION_ENABLED_VALUE = 0x0027


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


def checked_rotary_ramp(acceleration: float, deceleration: float, maximum: float) -> tuple[float, float]:
    if not all(math.isfinite(value) and 0.0 < value <= maximum for value in (acceleration, deceleration)):
        raise ValueError(f"Acceleration and deceleration must be within 0..{maximum:.1f} rpm/s")
    return acceleration, deceleration


def aligned_motor4_position_mm(requested_mm: float) -> float:
    """Validate and align Motor 4 to its 0.0001 mm hardware count."""
    if not math.isfinite(requested_mm):
        raise ValueError("Motor 4 position must be a finite number")
    if not MOTOR4_MIN_MM <= requested_mm <= MOTOR4_MAX_MM:
        raise ValueError("Motor 4 position must be 0..20 mm")
    aligned_mm = round(requested_mm / MOTOR4_INCREMENT_MM) * MOTOR4_INCREMENT_MM
    if not MOTOR4_MIN_MM <= aligned_mm <= MOTOR4_MAX_MM:
        raise ValueError("Aligned Motor 4 position is outside the guarded range")
    return aligned_mm


def aligned_motor3_position_mm(requested_mm: float) -> float:
    """Validate and align Motor 3 to its 0.0001 mm hardware count."""
    if not math.isfinite(requested_mm):
        raise ValueError("Motor 3 position must be a finite number")
    if not MOTOR3_MIN_MM <= requested_mm <= MOTOR3_MAX_MM:
        raise ValueError("Motor 3 position must be 0..20 mm")
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


def checked_motor6_distance_mm(requested_mm: float) -> float:
    if not math.isfinite(requested_mm) or not 0.0 < requested_mm <= MOTOR6_STEP_MAX_MM:
        raise ValueError(
            f"Motor 6 distance step must be within 0..{MOTOR6_STEP_MAX_MM:.0f} mm"
        )
    return requested_mm


class YuzuOperatorNode(Node):
    def __init__(self) -> None:
        super().__init__("yuzu_operator_gui")
        self.axis1_publisher = self.create_publisher(Float64MultiArray, AXIS1_TOPIC, 10)
        self.axis2_publisher = self.create_publisher(Float64MultiArray, AXIS2_TOPIC, 10)
        self.motor3_publisher = self.create_publisher(Float64, MOTOR3_TOPIC, 10)
        self.motor4_publisher = self.create_publisher(Float64, MOTOR4_TOPIC, 10)
        self.motor5_publisher = self.create_publisher(Float64, MOTOR5_TOPIC, 10)
        self.motor5_zero_publisher = self.create_publisher(Empty, MOTOR5_SET_ZERO_TOPIC, 10)
        self.motor5_manual_step_publisher = self.create_publisher(Float64, MOTOR5_MANUAL_STEP_TOPIC, 10)
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
        self.motor6_position_rad = None
        self.motor6_velocity_rad_s = None
        self.drive_status = {f"motor{motor}": None for motor in range(1, 7)}
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
            if index < len(message.position) and math.isfinite(message.position[index]):
                self.motor6_position_rad = message.position[index]
                self._mark_feedback("motor6")
            if index < len(message.velocity) and math.isfinite(message.velocity[index]):
                self.motor6_velocity_rad_s = message.velocity[index]
                self._mark_feedback("motor6")

    def _dynamic_joint_state(self, message: DynamicJointState) -> None:
        channels = (
            ("motor1_motor2", "motor3_position", "motor3_position_m", "motor3"),
            ("motor4_joint", "motor5_position", "motor5_position_rad", "motor5"),
            ("motor4_joint", "motor6_position", "motor6_position_rad", "motor6"),
            ("motor1_motor2", "motor1_status", "drive_status[1]", "status1"),
            ("motor1_motor2", "motor2_status", "drive_status[2]", "status2"),
            ("motor1_motor2", "motor3_status", "drive_status[3]", "status3"),
            ("motor4_joint", "motor4_status", "drive_status[4]", "status4"),
            ("motor4_joint", "motor5_status", "drive_status[5]", "status5"),
            ("motor4_joint", "motor6_status", "drive_status[6]", "status6"),
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
                if attribute.startswith("drive_status["):
                    index = int(attribute[-2])
                    self.drive_status[f"motor{index}"] = int(position)
                    self._mark_feedback(f"motor{index}")
                else:
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

    def publish_axis2_profile(self, rpm: float, acceleration: float, deceleration: float) -> None:
        message = Float64MultiArray()
        message.data = [rpm, acceleration, deceleration]
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

    def publish_motor5_manual_step(self, step_deg: float) -> None:
        message = Float64()
        message.data = step_deg
        self.motor5_manual_step_publisher.publish(message)

    def set_motor5_zero(self) -> None:
        self.motor5_zero_publisher.publish(Empty())

    def publish_motor6(self, rpm: float) -> None:
        message = Float64MultiArray()
        message.data = [rpm]
        self.motor6_publisher.publish(message)

    def publish_motor6_profile(self, rpm: float, acceleration: float, deceleration: float) -> None:
        message = Float64MultiArray()
        message.data = [rpm, acceleration, deceleration]
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
        self.motor6_distance_active = False
        self.motor6_distance_target_rad = None
        self.motor6_distance_deceleration_mm_s2 = None
        self.motor5_origin_rad = None

        root.title("Yuzu Peeler — Motor Operator")
        root.geometry("1500x900")
        root.minsize(1050, 760)
        root.protocol("WM_DELETE_WINDOW", self.close)

        style = ttk.Style()
        style.configure("Title.TLabel", font=("Sans", 18, "bold"))
        style.configure("Status.TLabel", font=("Sans", 11, "bold"))
        style.configure("Banner.TLabel", font=("Sans", 12, "bold"))

        scroll_container = ttk.Frame(root)
        scroll_container.pack(fill="both", expand=True)
        canvas = tk.Canvas(scroll_container, highlightthickness=0)
        scrollbar = ttk.Scrollbar(scroll_container, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        outer = ttk.Frame(canvas, padding=16)
        canvas_window = canvas.create_window((0, 0), window=outer, anchor="nw")
        outer.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(canvas_window, width=event.width))
        canvas.bind_all("<MouseWheel>", lambda event: canvas.yview_scroll(int(-event.delta / 120), "units"))
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

        notebook = ttk.Notebook(outer)
        notebook.pack(fill="both", expand=True)
        manual_tab = ttk.Frame(notebook, padding=4)
        operation_tab = ttk.Frame(notebook, padding=12)
        notebook.add(manual_tab, text="Manual control")
        notebook.add(operation_tab, text="Operation sequence")

        panels = ttk.Frame(manual_tab)
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
        self._build_operation_tab(operation_tab)

        self.details_visible = False
        self.details_button = ttk.Button(
            manual_tab, text="Show motion details ▾", command=self.toggle_motion_details
        )
        self.details_button.pack(anchor="w", pady=(10, 0))
        self.details_frame = self._build_motion_details(manual_tab)

        self.status_text = tk.StringVar(value="Commands are sent to guarded backends; completion is not reported by this GUI.")
        self.command_status_label = ttk.Label(
            outer, textvariable=self.status_text, wraplength=900
        )
        self.command_status_label.pack(
            fill="x", pady=(10, 0)
        )
        self.root.after(20, self.tick)

    def _build_operation_tab(self, parent: ttk.Frame) -> None:
        ttk.Label(
            parent,
            text="Peeling operation — execute one guarded stage at a time; verify motion before pressing Next step.",
            style="Status.TLabel",
            wraplength=900,
        ).pack(anchor="w", pady=(0, 10))
        settings = ttk.LabelFrame(parent, text="Operator inputs", padding=10)
        settings.pack(fill="x", pady=(0, 10))
        fields = (
            ("Step 1 · Motor 6 positioning distance (mm)", "30.0"),
            ("Step 2 · Motor 1 approach position (mm)", "300.0"),
            ("Step 4.2 · Motor 3 feed-in position (mm)", "5.0"),
            ("Step 4.2 · Motor 4 feed-in position (mm)", "5.0"),
            ("Step 5 · Motor 3 gripping position (mm)", "2.0"),
            ("Step 5 · Motor 4 gripping position (mm)", "2.0"),
            ("Step 4.1 · Motor 2 CCW speed (rpm)", "250.0"),
            ("Step 6 · Motor 1 home position (mm)", "400.0"),
        )
        self.operation_entries = {}
        for row, (label, default) in enumerate(fields):
            ttk.Label(settings, text=label).grid(row=row // 2, column=(row % 2) * 2, sticky="w", padx=4, pady=3)
            entry = ttk.Entry(settings, width=12)
            entry.insert(0, default)
            entry.grid(row=row // 2, column=(row % 2) * 2 + 1, sticky="e", padx=4, pady=3)
            self.operation_entries[label] = entry
        steps = (
            "1. Yuzu positioning — Motor 6 forward distance",
            "2. Yuzu placement — Motor 1 approach",
            "3. Peeler positioning — Motor 5 to 90°",
            "4.1. Motor 2 start rotating CCW",
            "4.2. Motors 3/4 feed in",
            "4.3. Motor 5 rotate CW back 90°; stop Motor 2",
            "5. Yuzu gripping — Motors 3/4 gripping distances",
            "6. Feed motion — Motor 1 home position",
            "7. Yuzu release — Motors 3/4 home",
            "8. Waiting for next cycle — stop Motor 2",
        )
        self.operation_steps = steps
        self.operation_step_index = 0
        self.operation_step_labels = []
        step_list = ttk.LabelFrame(parent, text="Sequence steps", padding=8)
        step_list.pack(fill="x", pady=(0, 8))
        for step in steps:
            label = ttk.Label(step_list, text=step)
            label.pack(anchor="w", pady=1)
            self.operation_step_labels.append(label)
        self.operation_step_text = tk.StringVar()
        ttk.Label(parent, textvariable=self.operation_step_text, wraplength=900).pack(anchor="w", pady=4)
        buttons = ttk.Frame(parent)
        buttons.pack(fill="x", pady=(8, 0))
        self.operation_next_button = ttk.Button(buttons, text="Execute next step", command=self.operation_next)
        self.operation_next_button.pack(side="left")
        ttk.Button(buttons, text="Reset sequence", command=self.operation_reset).pack(side="left", padx=8)
        ttk.Button(buttons, text="Stop motors", command=self.stop_rotary_motors).pack(side="right")
        self._update_operation_step_text()

    def _operation_value(self, label: str) -> str:
        return self.operation_entries[label].get()

    def _set_entry_value(self, entry: ttk.Entry, value: str) -> None:
        entry.delete(0, tk.END)
        entry.insert(0, value)

    def _update_operation_step_text(self) -> None:
        for index, label in enumerate(self.operation_step_labels):
            if index == self.operation_step_index:
                label.configure(foreground="#0b5cad", font=("Sans", 10, "bold"))
            else:
                label.configure(foreground="#8a8a8a", font=("Sans", 10, "normal"))
        if self.operation_step_index >= len(self.operation_steps):
            self.operation_step_text.set("Sequence complete — reset to run another cycle.")
            self.operation_next_button.configure(state="disabled")
        else:
            self.operation_step_text.set(
                f"Ready: {self.operation_steps[self.operation_step_index]}"
            )
            self.operation_next_button.configure(state="normal")

    def operation_reset(self) -> None:
        self.operation_step_index = 0
        self._update_operation_step_text()
        self.status_text.set("Operation sequence reset; no motion command was sent.")

    def operation_next(self) -> None:
        step = self.operation_step_index
        if step == 0:
            self._set_entry_value(self.motor6_distance_entry, self._operation_value("Step 1 · Motor 6 positioning distance (mm)"))
            self.step_motor6(1)
        elif step == 1:
            self._set_entry_value(self.axis1_entry, self._operation_value("Step 2 · Motor 1 approach position (mm)"))
            self.move_axis1()
        elif step == 2:
            self._set_entry_value(self.motor5_entry, "90.0")
            self.motor5_direction.set(1)
            self.move_motor5()
        elif step == 3:
            self._set_entry_value(self.axis2_entry, self._operation_value("Step 4.1 · Motor 2 CCW speed (rpm)"))
            self.axis2_direction.set(-1)
            self.start_axis2()
        elif step == 4:
            self._set_entry_value(self.motor3_entry, self._operation_value("Step 4.2 · Motor 3 feed-in position (mm)"))
            self.move_motor3()
            self._set_entry_value(self.motor4_entry, self._operation_value("Step 4.2 · Motor 4 feed-in position (mm)"))
            self.move_motor4()
        elif step == 5:
            self._set_entry_value(self.motor5_entry, "0.0")
            self.motor5_direction.set(1)
            self.move_motor5()
            self.stop_axis2()
        elif step == 6:
            self._set_entry_value(self.motor3_entry, self._operation_value("Step 5 · Motor 3 gripping position (mm)"))
            self.move_motor3()
            self._set_entry_value(self.motor4_entry, self._operation_value("Step 5 · Motor 4 gripping position (mm)"))
            self.move_motor4()
        elif step == 7:
            self._set_entry_value(self.axis1_entry, self._operation_value("Step 6 · Motor 1 home position (mm)"))
            self.move_axis1()
        elif step == 8:
            self.home_motor3()
            self.home_motor4()
        elif step == 9:
            self.stop_axis2()
        self.operation_step_index += 1
        self._update_operation_step_text()

    def _build_motor1(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Motor 1 — Vertical position", padding=12)
        frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        ttk.Label(frame, text="Limits: 0–400 mm").pack(anchor="w")
        ttk.Label(frame, text="Step size: 0.0012 mm hardware count · default 1 mm").pack(anchor="w")
        ttk.Label(frame, text="Speed: 15 mm/s · stage-test the full stroke").pack(anchor="w")
        self.axis1_feedback = tk.StringVar(value="Feedback: —")
        ttk.Label(frame, textvariable=self.axis1_feedback, style="Status.TLabel").pack(
            anchor="w", pady=(10, 12)
        )
        row = ttk.Frame(frame)
        row.pack(fill="x")
        ttk.Label(row, text="Target (mm)").pack(side="left")
        self.axis1_entry = ttk.Entry(row, width=12)
        self.axis1_entry.insert(0, "0.0")
        self.axis1_entry.pack(side="right")
        step_row = ttk.Frame(frame); step_row.pack(fill="x", pady=(4, 0))
        ttk.Label(step_row, text="Step (mm)").pack(side="left")
        self.axis1_step_entry = ttk.Entry(step_row, width=12); self.axis1_step_entry.insert(0, "1.0"); self.axis1_step_entry.pack(side="right")
        for label, direction in ((">", 1), ("<", -1)):
            button = ttk.Button(step_row, text=label, width=3, command=lambda d=direction: self.step_axis1(d))
            button.pack(side="right", padx=(2, 0) if direction > 0 else (0, 0))
            self.step_buttons[1].append(button)
        self.axis1_move = ttk.Button(frame, text="Move Motor 1", command=self.move_axis1)
        self.axis1_move.pack(fill="x", pady=(12, 6))
        self.axis1_park = ttk.Button(
            frame, text="Return to 0 mm", command=self.park_axis1
        )
        self.axis1_park.pack(fill="x")

    def _build_motor2(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Motor 2 — Yuzu rotation", padding=12)
        frame.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        ttk.Label(frame, text="Limits: 0–250 rpm project · 416 rpm drive max").pack(anchor="w")
        ttk.Label(frame, text="Step size: continuous rpm command · default 250 rpm").pack(anchor="w")
        self.axis2_feedback = tk.StringVar(value="Feedback: —")
        ttk.Label(frame, textvariable=self.axis2_feedback, style="Status.TLabel").pack(
            anchor="w", pady=(10, 12)
        )
        row = ttk.Frame(frame)
        row.pack(fill="x")
        ttk.Label(row, text="Speed (rpm)").pack(side="left")
        self.axis2_entry = ttk.Entry(row, width=12)
        self.axis2_entry.insert(0, "250.0")
        self.axis2_entry.pack(side="right")
        profile = ttk.Frame(frame)
        profile.pack(fill="x", pady=(4, 0))
        ttk.Label(profile, text="Acc / dec (rpm/s)").pack(side="left")
        self.axis2_accel_entry = ttk.Entry(profile, width=7)
        self.axis2_accel_entry.insert(0, "250.0")
        self.axis2_accel_entry.pack(side="right", padx=(3, 0))
        self.axis2_decel_entry = ttk.Entry(profile, width=7)
        self.axis2_decel_entry.insert(0, "250.0")
        self.axis2_decel_entry.pack(side="right")
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
        ttk.Label(frame, text="Limits: 0–20 mm absolute").pack(anchor="w")
        ttk.Label(frame, text="Step size: 0.0001 mm hardware count · default 1 mm").pack(anchor="w")
        ttk.Label(frame, text="Move only after confirming the mechanism is clear").pack(anchor="w")
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
        step_row = ttk.Frame(frame); step_row.pack(fill="x", pady=(4, 0))
        ttk.Label(step_row, text="Step (mm)").pack(side="left")
        self.motor4_step_entry = ttk.Entry(step_row, width=12); self.motor4_step_entry.insert(0, "1.0"); self.motor4_step_entry.pack(side="right")
        for label, direction in ((">", 1), ("<", -1)):
            button = ttk.Button(step_row, text=label, width=3, command=lambda d=direction: self.step_motor4(d))
            button.pack(side="right", padx=(2, 0) if direction > 0 else (0, 0))
            self.step_buttons[4].append(button)
        self.motor4_move = ttk.Button(frame, text="Move Motor 4", command=self.move_motor4)
        self.motor4_move.pack(fill="x", pady=(12, 6))
        self.motor4_home = ttk.Button(frame, text="Return to 0 mm", command=self.home_motor4)
        self.motor4_home.pack(fill="x")

    def _build_motor6(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Motor 6 — Conveyor", padding=12)
        frame.grid(row=1, column=2, sticky="nsew", padx=4, pady=4)
        ttk.Label(frame, text="Limits: 0–90 mm/s belt speed").pack(anchor="w")
        ttk.Label(frame, text="Distance mode: relative conveyor steps using encoder position").pack(anchor="w")
        ttk.Label(frame, text="+ direction: verified forward · use controlled stop").pack(anchor="w")
        self.motor6_feedback = tk.StringVar(value="Feedback: —")
        ttk.Label(frame, textvariable=self.motor6_feedback, style="Status.TLabel").pack(
            anchor="w", pady=(10, 12)
        )
        row = ttk.Frame(frame)
        row.pack(fill="x")
        ttk.Label(row, text="Belt speed (mm/s)").pack(side="left")
        self.motor6_entry = ttk.Entry(row, width=12)
        self.motor6_entry.insert(0, "50.0")
        self.motor6_entry.pack(side="right")
        distance_row = ttk.Frame(frame)
        distance_row.pack(fill="x", pady=(4, 0))
        ttk.Label(distance_row, text="Distance step (mm)").pack(side="left")
        self.motor6_distance_entry = ttk.Entry(distance_row, width=12)
        self.motor6_distance_entry.insert(0, "30.0")
        self.motor6_distance_entry.pack(side="right")
        profile = ttk.Frame(frame)
        profile.pack(fill="x", pady=(4, 0))
        ttk.Label(profile, text="Acc / dec (rpm/s)").pack(side="left")
        self.motor6_accel_entry = ttk.Entry(profile, width=7)
        self.motor6_accel_entry.insert(0, "250.0")
        self.motor6_accel_entry.pack(side="right", padx=(3, 0))
        self.motor6_decel_entry = ttk.Entry(profile, width=7)
        self.motor6_decel_entry.insert(0, "250.0")
        self.motor6_decel_entry.pack(side="right")
        step_row = ttk.Frame(frame)
        step_row.pack(fill="x", pady=(8, 0))
        ttk.Label(step_row, text="Move one step").pack(side="left")
        for label, direction in ((">", 1), ("<", -1)):
            button = ttk.Button(
                step_row, text=label, width=3,
                command=lambda d=direction: self.step_motor6(d),
            )
            button.pack(side="right", padx=(2, 0) if direction > 0 else (0, 0))
            self.step_buttons[6].append(button)
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
        ttk.Label(frame, text="Limits: ±180° after zero is captured").pack(anchor="w")
        ttk.Label(frame, text="Step size: 0.0018° hardware count · default 90°").pack(anchor="w")
        ttk.Label(frame, text="Jog manually with < / >, stop, then set current position as zero").pack(anchor="w")
        self.motor5_feedback = tk.StringVar(value="Feedback: —")
        ttk.Label(frame, textvariable=self.motor5_feedback, style="Status.TLabel").pack(
            anchor="w", pady=(10, 8)
        )
        ttk.Label(frame, text="−180° from zero     0°     +180° from zero").pack(anchor="w")
        self.motor5_angle_bar = ttk.Progressbar(frame, maximum=360.0, mode="determinate")
        self.motor5_angle_bar.pack(fill="x", pady=(0, 8))
        self.motor5_set_zero = ttk.Button(
            frame, text="Set current position as zero", command=self.set_motor5_zero
        )
        self.motor5_set_zero.pack(fill="x", pady=(0, 8))
        row = ttk.Frame(frame)
        row.pack(fill="x")
        ttk.Label(row, text="Angle (deg)").pack(side="left")
        self.motor5_entry = ttk.Entry(row, width=10)
        self.motor5_entry.insert(0, "0.0")
        self.motor5_entry.pack(side="right")
        step_row = ttk.Frame(frame); step_row.pack(fill="x", pady=(4, 0))
        ttk.Label(step_row, text="Step (deg)").pack(side="left")
        self.motor5_step_entry = ttk.Entry(step_row, width=10); self.motor5_step_entry.insert(0, "90.0"); self.motor5_step_entry.pack(side="right")
        for label, direction in ((">", 1), ("<", -1)):
            button = ttk.Button(step_row, text=label, width=3, command=lambda d=direction: self.step_motor5(d))
            button.pack(side="right", padx=(2, 0) if direction > 0 else (0, 0))
            self.step_buttons[5].append(button)
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
        ttk.Label(frame, text="Limits: 0–20 mm absolute").pack(anchor="w")
        ttk.Label(frame, text="Step size: 0.0001 mm hardware count · default 1 mm").pack(anchor="w")
        ttk.Label(frame, text="Move only after confirming the mechanism is clear").pack(anchor="w")
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
        step_row = ttk.Frame(frame); step_row.pack(fill="x", pady=(4, 0))
        ttk.Label(step_row, text="Step (mm)").pack(side="left")
        self.motor3_step_entry = ttk.Entry(step_row, width=12); self.motor3_step_entry.insert(0, "1.0"); self.motor3_step_entry.pack(side="right")
        for label, direction in ((">", 1), ("<", -1)):
            button = ttk.Button(step_row, text=label, width=3, command=lambda d=direction: self.step_motor3(d))
            button.pack(side="right", padx=(2, 0) if direction > 0 else (0, 0))
            self.step_buttons[3].append(button)
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
                f"±{MOTOR6_GUI_MAX_MM_S:.0f} mm/s",
                f"{MOTOR6_GUI_MAX_MM_S:.0f} mm/s max",
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
        ttk.Label(
            frame,
            text=(
                "Hardware envelope: Motor 1 stroke 400 mm; Motors 3/4 catalog stroke 30 mm "
                "(project guard 20 mm); Motor 2 output speed 416 rpm (project guard 250 rpm); "
                "Motor 5 output speed 150 rpm with multi-turn encoder and no intrinsic angular stop; "
                "Motor 6 drive maximum 60 rpm ≈94.25 mm/s on the 30 mm pulley (GUI guard 90 mm/s). "
                "Acceleration limits are software commissioning caps, not manufacturer ratings."
            ),
            wraplength=1200,
            justify="left",
        ).grid(row=len(rows) + 2, column=0, columnspan=5, sticky="w", pady=(7, 0))
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
        entry.insert(0, f"{value:.4f}")
        callback()

    def step_axis1(self, direction):
        self.step_target(self.axis1_entry, self.axis1_step_entry, direction, AXIS1_MIN_MM, AXIS1_MAX_MM, self.move_axis1)

    def step_motor3(self, direction):
        self.step_target(self.motor3_entry, self.motor3_step_entry, direction, MOTOR3_MIN_MM, MOTOR3_MAX_MM, self.move_motor3)

    def step_motor4(self, direction):
        self.step_target(self.motor4_entry, self.motor4_step_entry, direction, MOTOR4_MIN_MM, MOTOR4_MAX_MM, self.move_motor4)

    def step_motor5(self, direction):
        if self.motor5_origin_rad is None:
            try:
                step_deg = abs(float(self.motor5_step_entry.get()))
                if not math.isfinite(step_deg) or step_deg <= 0.0 or step_deg > MOTOR5_MAX_ANGLE_DEG:
                    raise ValueError
            except ValueError:
                messagebox.showerror("Invalid Motor 5 step", "Enter a step between 0 and 90 degrees.")
                return
            self.node.publish_motor5_manual_step(direction * step_deg)
            self.status_text.set(
                f"Motor 5 manual jog sent: {direction * step_deg:+.1f}° · stop before setting zero"
            )
            return
        self.step_target(
            self.motor5_entry,
            self.motor5_step_entry,
            direction,
            0.0,
            MOTOR5_MAX_ANGLE_DEG,
            self.move_motor5,
        )

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
            acceleration, deceleration = checked_rotary_ramp(
                float(self.axis2_accel_entry.get()),
                float(self.axis2_decel_entry.get()),
                AXIS2_ACCELERATION_RPM_S,
            )
        except ValueError as error:
            messagebox.showerror("Invalid Motor 2 command", str(error))
            return
        self.axis2_command_rpm = rpm * self.axis2_direction.get()
        self.axis2_running = rpm > 0.0
        self.node.publish_axis2_profile(self.axis2_command_rpm, acceleration, deceleration)
        self.status_text.set(
            f"Motor 2 speed command sent: {self.axis2_command_rpm:.1f} rpm · "
            f"acc {acceleration:.1f} / dec {deceleration:.1f} rpm/s"
        )

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
            acceleration, deceleration = checked_rotary_ramp(
                float(self.motor6_accel_entry.get()),
                float(self.motor6_decel_entry.get()),
                MOTOR6_ACCELERATION_RPM_S,
            )
        except ValueError as error:
            messagebox.showerror("Invalid Motor 6 command", str(error))
            return
        self.motor6_command_rpm = rpm * self.motor6_direction.get()
        self.motor6_running = rpm > 0.0
        self.motor6_distance_active = False
        self.motor6_distance_target_rad = None
        self.motor6_distance_deceleration_mm_s2 = None
        self.node.publish_motor6_profile(self.motor6_command_rpm, acceleration, deceleration)
        belt_speed_mm_s = (
            abs(self.motor6_command_rpm) * math.pi * MOTOR6_PULLEY_DIAMETER_MM / 60.0
        )
        self.status_text.set(
            f"Motor 6 conveyor command sent: {belt_speed_mm_s:.2f} mm/s · "
            f"acc {acceleration:.1f} / dec {deceleration:.1f} rpm/s"
        )

    def stop_motor6(self) -> None:
        self.motor6_running = False
        self.motor6_command_rpm = 0.0
        self.motor6_distance_active = False
        self.motor6_distance_target_rad = None
        self.motor6_distance_deceleration_mm_s2 = None
        self.node.publish_motor6(0.0)
        self.status_text.set("Motor 6 stop requested; guarded deceleration is active.")

    def step_motor6(self, direction: int) -> None:
        if self.node.motor6_publisher.get_subscription_count() == 0:
            messagebox.showerror("Motor 6 unavailable", "Motor 6 guarded backend is not running.")
            return
        if self.node.motor6_position_rad is None or not self.node.feedback_is_fresh_for("motor6"):
            messagebox.showerror(
                "Motor 6 feedback unavailable",
                "Wait for fresh conveyor position feedback before stepping.",
            )
            return
        try:
            distance_mm = checked_motor6_distance_mm(float(self.motor6_distance_entry.get()))
            speed_rpm = checked_motor6_speed_mm_s(float(self.motor6_entry.get()))
            acceleration, deceleration = checked_rotary_ramp(
                float(self.motor6_accel_entry.get()),
                float(self.motor6_decel_entry.get()),
                MOTOR6_ACCELERATION_RPM_S,
            )
        except ValueError as error:
            messagebox.showerror("Invalid Motor 6 step", str(error))
            return
        self.motor6_distance_target_rad = (
            self.node.motor6_position_rad
            + direction * distance_mm / (MOTOR6_PULLEY_DIAMETER_MM / 2.0)
        )
        self.motor6_command_rpm = direction * speed_rpm
        self.motor6_running = True
        self.motor6_distance_active = True
        self.motor6_distance_deceleration_mm_s2 = (
            deceleration * math.pi * MOTOR6_PULLEY_DIAMETER_MM / 60.0
        )
        self.node.publish_motor6_profile(self.motor6_command_rpm, acceleration, deceleration)
        self.status_text.set(
            f"Motor 6 step started: {direction * distance_mm:+.2f} mm · "
            "stopping at encoder target"
        )

    def stop_rotary_motors(self) -> None:
        self.axis2_running = False
        self.axis2_command_rpm = 0.0
        self.motor6_running = False
        self.motor6_command_rpm = 0.0
        self.motor6_distance_active = False
        self.motor6_distance_target_rad = None
        self.motor6_distance_deceleration_mm_s2 = None
        for _ in range(5):
            self.node.publish_axis2(0.0)
            self.node.publish_motor6(0.0)
        self.status_text.set("Motor 2 and Motor 6 guarded stops requested.")

    def tick(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)
        backend = {
            "motor1": self.node.axis1_publisher.get_subscription_count() > 0
            or self.node.feedback_times["motor1"] is not None,
            "motor2": self.node.axis2_publisher.get_subscription_count() > 0
            or self.node.feedback_times["motor2"] is not None,
            "motor3": self.node.motor3_publisher.get_subscription_count() > 0
            or self.node.feedback_times["motor3"] is not None,
            "motor4": self.node.motor4_publisher.get_subscription_count() > 0
            or self.node.feedback_times["motor4"] is not None,
            "motor5": (
                self.node.motor5_publisher.get_subscription_count() > 0
                and self.node.motor5_zero_publisher.get_subscription_count() > 0
                and self.node.motor5_manual_step_publisher.get_subscription_count() > 0
            ) or self.node.feedback_times["motor5"] is not None,
            "motor6": self.node.motor6_publisher.get_subscription_count() > 0
            or self.node.feedback_times["motor6"] is not None,
        }
        fresh = {
            motor: self.node.feedback_is_fresh_for(motor) for motor in backend
        }
        status_ready = {
            motor: (
                self.node.drive_status[motor] is not None
                and (self.node.drive_status[motor] & OPERATION_ENABLED_MASK)
                == OPERATION_ENABLED_VALUE
            )
            for motor in backend
        }
        feedback_ready = {motor: backend[motor] and fresh[motor] for motor in backend}
        ready = {
            motor: feedback_ready[motor] and status_ready[motor] for motor in backend
        }

        if self.axis2_running and not ready["motor2"]:
            self.axis2_running = False
            self.axis2_command_rpm = 0.0
            for _ in range(5):
                self.node.publish_axis2(0.0)
            self.status_text.set("Motor 2 feedback lost; repeated controlled stop commands sent.")
        if self.motor6_running and not ready["motor6"]:
            self.motor6_running = False
            self.motor6_command_rpm = 0.0
            self.motor6_distance_active = False
            self.motor6_distance_target_rad = None
            self.motor6_distance_deceleration_mm_s2 = None
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
            button.configure(state="normal" if motor5_ready else "disabled")
        for button in self.step_buttons[6]:
            button.configure(state="normal" if motor6_ready else "disabled")

        if self.axis2_running:
            self.node.publish_axis2(self.axis2_command_rpm)
        if self.motor6_running:
            if self.motor6_distance_active and self.motor6_distance_target_rad is not None:
                current = self.node.motor6_position_rad
                direction = 1.0 if self.motor6_command_rpm >= 0.0 else -1.0
                remaining_mm = (
                    direction
                    * (self.motor6_distance_target_rad - current)
                    * (MOTOR6_PULLEY_DIAMETER_MM / 2.0)
                    if current is not None
                    else math.inf
                )
                speed_mm_s = abs(self.node.motor6_velocity_rad_s or 0.0) * (
                    MOTOR6_PULLEY_DIAMETER_MM / 2.0
                )
                deceleration = self.motor6_distance_deceleration_mm_s2 or 1.0
                stopping_distance_mm = speed_mm_s * speed_mm_s / (2.0 * deceleration)
                reached = remaining_mm <= max(0.3, stopping_distance_mm)
                if reached:
                    self.motor6_running = False
                    self.motor6_command_rpm = 0.0
                    self.motor6_distance_active = False
                    self.motor6_distance_target_rad = None
                    self.node.publish_motor6(0.0)
                    self.status_text.set("Motor 6 distance step braking; guarded stop requested.")
                else:
                    self.node.publish_motor6(self.motor6_command_rpm)
            else:
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
                f"{state} · Actual: {motor4_mm:.3f} mm / 20 mm"
            )
            self.motor4_travel.configure(value=max(0.0, min(MOTOR4_MAX_MM, motor4_mm)))
        if self.node.motor3_position_m is None:
            self.motor3_feedback.set("Waiting for Motor 3 feedback")
            self.motor3_travel.configure(value=0.0)
        else:
            state = "FEEDBACK LIVE" if fresh["motor3"] else "FEEDBACK STALE"
            motor3_mm = self.node.motor3_position_m * 1000.0
            self.motor3_feedback.set(
                f"{state} · Actual: {motor3_mm:.3f} mm / 20 mm"
            )
            self.motor3_travel.configure(value=max(0.0, min(MOTOR3_MAX_MM, motor3_mm)))
        if self.node.motor6_velocity_rad_s is None:
            self.motor6_feedback.set("Waiting for Motor 6 feedback")
        else:
            rpm = self.node.motor6_velocity_rad_s * 30.0 / math.pi
            belt_speed_mm_s = rpm * math.pi * MOTOR6_PULLEY_DIAMETER_MM / 60.0
            state = "FEEDBACK LIVE" if fresh["motor6"] else "FEEDBACK STALE"
            position_text = "Encoder travel: unavailable"
            if self.node.motor6_position_rad is not None:
                position_text = (
                    f"Encoder travel: {self.node.motor6_position_rad * MOTOR6_PULLEY_DIAMETER_MM / 2.0:.2f} mm"
                )
            self.motor6_feedback.set(
                f"{state} · Actual speed: {belt_speed_mm_s:.2f} mm/s\n{position_text}"
            )
        if self.node.motor5_position_rad is None:
            self.motor5_feedback.set("Waiting for Motor 5 feedback")
        elif self.motor5_origin_rad is None:
            state = "FEEDBACK LIVE" if fresh["motor5"] else "FEEDBACK STALE"
            self.motor5_feedback.set(f"{state} · Set runtime zero after confirming stopped")
            self.motor5_angle_bar.configure(value=180.0)
        else:
            relative_deg = math.degrees(
                self.node.motor5_position_rad - self.motor5_origin_rad
            )
            state = "FEEDBACK LIVE" if fresh["motor5"] else "FEEDBACK STALE"
            self.motor5_feedback.set(f"{state} · Relative: {relative_deg:+.3f}°")
            self.motor5_angle_bar.configure(value=max(0.0, min(360.0, relative_deg + 180.0)))

        fresh_count = sum(feedback_ready.values())
        connected_count = sum(backend.values())
        status_count = sum(status_ready.values())
        self.connection_text.set(
            f"Control paths: {connected_count}/6 · feedback: {fresh_count}/6 · drives enabled: {status_count}/6"
        )
        if status_count == len(backend) and self.operation_step_index < len(self.operation_steps):
            self.operation_next_button.configure(state="normal")
        else:
            self.operation_next_button.configure(state="disabled")
        if connected_count == 0:
            banner, color = "BACKEND OFFLINE · motor commands disabled", "#5b6470"
        elif fresh_count != connected_count:
            banner, color = "FEEDBACK MISSING OR STALE · check each motor panel", "#b54708"
        elif status_count != len(backend):
            not_ready = [str(motor) for motor in backend if not status_ready[motor]]
            banner, color = (
                f"DRIVES NOT READY · waiting for operation enabled (0x0567): Motor {', '.join(not_ready)}",
                "#b54708",
            )
        else:
            banner, color = (
                "ALL DRIVES READY · operation enabled (0x0567) · commands unlocked",
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
