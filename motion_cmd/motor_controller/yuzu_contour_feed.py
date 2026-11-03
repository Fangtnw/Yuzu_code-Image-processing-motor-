"""Calculate guarded Motor 3/4 feed targets from a complete inset contour."""

import math


ANGLE_STEP_DEG = 5
ANGLES = tuple(range(0, 360, ANGLE_STEP_DEG))
SLIDE_MIN_MM = 0.0
SLIDE_MAX_MM = 20.0
SLIDE_INCREMENT_MM = 0.0001


class ContourFeedProfile:
    """72 measured triples [angle_deg, dx_mm, dy_mm], zero up, CW positive."""

    def __init__(self, data, source_inset_mm=3.0, peel_depth_mm=0.0):
        self.source_inset_mm = float(source_inset_mm)
        self.peel_depth_mm = float(peel_depth_mm)
        if not math.isfinite(self.source_inset_mm) or self.source_inset_mm < 0:
            raise ValueError("Source inset must be finite and nonnegative")
        if not math.isfinite(self.peel_depth_mm) or self.peel_depth_mm < 0:
            raise ValueError("Peel depth must be finite and nonnegative")
        if len(data) != 216:
            raise ValueError("Expected 72 contour points: [angle, dx_mm, dy_mm] x 72")
        radii = {}
        for i in range(0, len(data), 3):
            angle, x, y = map(float, data[i:i + 3])
            if not all(math.isfinite(v) for v in (angle, x, y)):
                raise ValueError("Contour point contains a non-finite value")
            if angle not in ANGLES or angle in radii:
                raise ValueError("Expected unique angles 0..355 at 5-degree spacing")
            radius = math.hypot(x, y)
            if radius <= 0:
                raise ValueError("Contour radius must be positive")
            direction = math.degrees(math.atan2(x, y)) % 360
            if abs((direction - angle + 180) % 360 - 180) > 3:
                raise ValueError("Contour coordinates disagree with their angle")
            radii[int(angle)] = radius
        if set(radii) != set(ANGLES):
            raise ValueError("Contour profile is missing an angle")
        self.radii_mm = radii

    def radius_mm(self, angle_deg):
        """Return a measured 5° ray; never infer a peeling radius."""
        angle = float(angle_deg)
        if not math.isfinite(angle):
            raise ValueError("Contour query angle must be finite")
        angle %= 360.0
        nearest = round(angle / ANGLE_STEP_DEG) * ANGLE_STEP_DEG
        if abs(angle - nearest) > 1e-6:
            raise ValueError("Peeling radius requested between measured 5-degree rays")
        return self.radii_mm[int(nearest) % 360]

    def slide_target_mm(self, angle_deg, blade_center_distance_mm=35.0):
        return self._target_from_radius(self.radius_mm(angle_deg), blade_center_distance_mm)

    def _target_from_radius(self, radius_mm, blade_center_distance_mm):
        distance = float(blade_center_distance_mm)
        if not math.isfinite(distance) or distance <= 0:
            raise ValueError("Blade-center distance must be finite and positive")
        # The camera publishes a 3 mm inward-shifted contour. Restore its
        # outer-surface radius, then add only the explicitly requested depth.
        target = distance - (radius_mm + self.source_inset_mm) + self.peel_depth_mm
        if not SLIDE_MIN_MM <= target <= SLIDE_MAX_MM:
            raise ValueError(f"Slide target {target:.3f} mm is outside 0..20 mm")
        return round(round(target / SLIDE_INCREMENT_MM) * SLIDE_INCREMENT_MM, 4)

    def averaged_slide_target_mm(self, first_angle_deg, mirror_angle_deg,
                                 blade_center_distance_mm=35.0):
        mean_radius = (self.radius_mm(first_angle_deg) +
                       self.radius_mm(mirror_angle_deg)) / 2.0
        return self._target_from_radius(mean_radius, blade_center_distance_mm)

    def targets_at_motor5(self, motor5_deg, camera_angle_direction=1,
                          motor3_center_distance_mm=35.0,
                          motor4_center_distance_mm=35.0):
        """Map a Motor 5 sweep target to the two opposing blade rays.

        The operator confirmed that the active blade rays increase from 0°
        and 180° as Motor 5 sweeps CCW from +90°. ``camera_angle_direction``
        remains adjustable for a changed camera mounting or drive direction.
        """
        motor5 = float(motor5_deg)
        if not math.isfinite(motor5) or not -10.0 <= motor5 <= 90.0:
            raise ValueError("Motor 5 sweep angle must be within -10..90 degrees")
        if camera_angle_direction not in (-1, 1):
            raise ValueError("Camera angle direction must be +1 or -1")
        travel_ccw = 90.0 - motor5
        motor3_angle = (camera_angle_direction * travel_ccw) % 360.0
        motor4_angle = (180.0 + camera_angle_direction * travel_ccw) % 360.0
        motor3_mirror_angle = (-camera_angle_direction * travel_ccw) % 360.0
        motor4_mirror_angle = (180.0 - camera_angle_direction * travel_ccw) % 360.0
        try:
            motor3_target = self.averaged_slide_target_mm(
                motor3_angle, motor3_mirror_angle, motor3_center_distance_mm)
        except ValueError as error:
            raise ValueError(
                f"Motor 3 at Motor 5 {motor5:+.0f} degrees, rays "
                f"{motor3_angle:g}/{motor3_mirror_angle:g}: {error}"
            ) from error
        try:
            motor4_target = self.averaged_slide_target_mm(
                motor4_angle, motor4_mirror_angle, motor4_center_distance_mm)
        except ValueError as error:
            raise ValueError(
                f"Motor 4 at Motor 5 {motor5:+.0f} degrees, rays "
                f"{motor4_angle:g}/{motor4_mirror_angle:g}: {error}"
            ) from error
        return {
            "motor5_deg": motor5,
            "motor3_contour_angle_deg": motor3_angle,
            "motor3_mirror_angle_deg": motor3_mirror_angle,
            "motor4_contour_angle_deg": motor4_angle,
            "motor4_mirror_angle_deg": motor4_mirror_angle,
            "motor3_target_mm": motor3_target,
            "motor4_target_mm": motor4_target,
        }
