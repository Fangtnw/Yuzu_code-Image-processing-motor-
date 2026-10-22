"""Signed target arithmetic for the Motor 5 GUI step buttons."""

import math


def stepped_motor5_target(magnitude_deg, selected_direction, step_deg,
                          button_direction, limit_deg=180.0):
    values = (magnitude_deg, step_deg, limit_deg)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Motor 5 angle and step must be finite")
    if not 0.0 <= magnitude_deg <= limit_deg:
        raise ValueError(f"Motor 5 angle must be 0..{limit_deg:g} degrees")
    if not 0.0 < step_deg <= limit_deg:
        raise ValueError(f"Motor 5 step must be greater than 0 and at most {limit_deg:g} degrees")
    if selected_direction not in (-1, 1) or button_direction not in (-1, 1):
        raise ValueError("Motor 5 direction must be +1 or -1")
    signed_target = selected_direction * magnitude_deg + button_direction * step_deg
    return max(-limit_deg, min(limit_deg, signed_target))
