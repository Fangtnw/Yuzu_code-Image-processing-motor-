"""Safe one-axis command builders for early AZD3A-KED ROS2 tests."""

from dataclasses import dataclass


@dataclass(frozen=True)
class AxisLimits:
    min_position_mm: float = -1000.0
    max_position_mm: float = 1000.0
    min_speed_mm_s: float = 0.001
    max_speed_mm_s: float = 1000.0
    min_rpm: float = -3000.0
    max_rpm: float = 3000.0
    min_angle_deg: float = -360000.0
    max_angle_deg: float = 360000.0


@dataclass(frozen=True)
class BasicCommand:
    topic: str
    message_type: str
    values: list[float]


def _require_range(name: str, value: float, minimum: float, maximum: float) -> None:
    if value < minimum or value > maximum:
        raise ValueError(f"{name} {value} outside [{minimum}, {maximum}]")


def _require_non_negative(name: str, value: float) -> None:
    if value < 0.0:
        raise ValueError(f"{name} must be >= 0.0")


def build_move_mm_command(
    position_mm: float,
    speed_mm_s: float,
    accel_mm_s2: float = 0.0,
    decel_mm_s2: float | None = None,
    limits: AxisLimits | None = None,
) -> BasicCommand:
    """Build the existing linear position payload: [position_mm, speed_mm_s, accel, decel]."""
    limits = limits or AxisLimits()
    decel = accel_mm_s2 if decel_mm_s2 is None else decel_mm_s2

    _require_range("position_mm", position_mm, limits.min_position_mm, limits.max_position_mm)
    _require_range("speed_mm_s", speed_mm_s, limits.min_speed_mm_s, limits.max_speed_mm_s)
    _require_non_negative("accel_mm_s2", accel_mm_s2)
    _require_non_negative("decel_mm_s2", decel)

    return BasicCommand(
        topic="motor_cmd",
        message_type="Float32MultiArray",
        values=[float(position_mm), float(speed_mm_s), float(accel_mm_s2), float(decel)],
    )


def build_spin_rpm_command(
    rpm: float,
    accel_rpm_s: float = 0.0,
    decel_rpm_s: float | None = None,
    limits: AxisLimits | None = None,
) -> BasicCommand:
    """Build the existing rotary speed payload: [rpm, accel_rpm_s, decel_rpm_s]."""
    limits = limits or AxisLimits()
    decel = accel_rpm_s if decel_rpm_s is None else decel_rpm_s

    _require_range("rpm", rpm, limits.min_rpm, limits.max_rpm)
    _require_non_negative("accel_rpm_s", accel_rpm_s)
    _require_non_negative("decel_rpm_s", decel)

    return BasicCommand(
        topic="motor_spin",
        message_type="Float32MultiArray",
        values=[float(rpm), float(accel_rpm_s), float(decel)],
    )


def build_angle_command(
    angle_deg: float,
    speed_rpm: float,
    accel_rpm_s: float = 0.0,
    decel_rpm_s: float | None = None,
    limits: AxisLimits | None = None,
) -> BasicCommand:
    """Build the existing rotary position payload: [angle_deg, speed_rpm, accel, decel]."""
    limits = limits or AxisLimits()
    decel = accel_rpm_s if decel_rpm_s is None else decel_rpm_s

    _require_range("angle_deg", angle_deg, limits.min_angle_deg, limits.max_angle_deg)
    _require_range("speed_rpm", speed_rpm, 0.001, limits.max_rpm)
    _require_non_negative("accel_rpm_s", accel_rpm_s)
    _require_non_negative("decel_rpm_s", decel)

    return BasicCommand(
        topic="motor_angle_cmd",
        message_type="Float32MultiArray",
        values=[float(angle_deg), float(speed_rpm), float(accel_rpm_s), float(decel)],
    )
