"""Motor 1 targets from a camera top-point offset at the home pose."""

from dataclasses import dataclass
import math


MOTOR1_COUNT_MM = 0.0012
MOTOR1_MIN_MM = 0.0
MOTOR1_MAX_MM = 400.0


def _finite_positive(name, value, allow_zero=False):
    value = float(value)
    if not math.isfinite(value) or (value < 0 if allow_zero else value <= 0):
        raise ValueError(f"{name} must be finite and {'nonnegative' if allow_zero else 'positive'}")
    return value


def _motor1_target(name, value):
    if not MOTOR1_MIN_MM <= value <= MOTOR1_MAX_MM:
        raise ValueError(f"{name} {value:.3f} mm is outside Motor 1's 0..400 mm range")
    return round(round(value / MOTOR1_COUNT_MM) * MOTOR1_COUNT_MM, 7)


@dataclass(frozen=True)
class Motor1CameraReference:
    """Map the detector's signed frame-center offset to a Motor 1 target.

    ``frame_center_position_mm`` is the Motor 1 encoder position where the
    holder tip/reference reaches the camera frame-center plane.
    The detector's red point is inset from the outer top contour. Positive
    detector offsets point upward in the image, whereas Motor 1 increases
    downward, so the default offset multiplier is -1.
    """

    frame_center_position_mm: float = 204.0
    home_position_mm: float = 0.0
    pierce_depth_mm: float = 3.0
    detector_inset_mm: float = 3.0
    allowance_mm: float = 0.0
    offset_multiplier: int = -1

    def targets(self, y_offset_mm, lift_mm=120.0):
        frame_position = _finite_positive("frame-center encoder position", self.frame_center_position_mm)
        home = _finite_positive("home position", self.home_position_mm, allow_zero=True)
        pierce = _finite_positive("pierce depth", self.pierce_depth_mm, allow_zero=True)
        inset = _finite_positive("detector inset", self.detector_inset_mm, allow_zero=True)
        allowance = _finite_positive("allowance", self.allowance_mm, allow_zero=True)
        offset = float(y_offset_mm)
        if not math.isfinite(offset):
            raise ValueError("camera Y offset must be finite")
        if self.offset_multiplier not in (-1, 1):
            raise ValueError("offset multiplier must be -1 or +1")
        lift = _finite_positive("lift distance", lift_mm)
        if not 100.0 <= lift <= 150.0:
            raise ValueError("Step 2 lift must be within the documented 100..150 mm range")
        _motor1_target("home", home)
        _motor1_target("frame center", frame_position)
        insertion = _motor1_target(
            "Step 1 insertion",
            frame_position + self.offset_multiplier * offset + pierce - inset + allowance,
        )
        lifted = _motor1_target("Step 2 lift", insertion - lift)
        return {
            "insertion_target_mm": insertion,
            "lift_target_mm": lifted,
            "home_position_mm": home,
            "frame_center_position_mm": frame_position,
            "y_offset_mm": offset,
            "pierce_depth_mm": pierce,
            "detector_inset_mm": inset,
            "allowance_mm": allowance,
            "lift_distance_mm": lift,
        }
