"""Measured 30-degree profile; no extrapolation across the missing red point."""

import math
from bisect import bisect_left


class ContourProfile:
    def __init__(self, data):
        if len(data) != 33:
            raise ValueError("Expected 11 yellow points: [angle, dx_mm, dy_mm] x 11")
        points = {}
        for i in range(0, len(data), 3):
            angle, x, y = map(float, data[i:i + 3])
            if not all(math.isfinite(v) for v in (angle, x, y)):
                raise ValueError("Contour contains non-finite values")
            if angle not in range(30, 360, 30) or angle in points:
                raise ValueError("Expected unique angles 30..330 at 30-degree spacing")
            radius = math.hypot(x, y)
            if radius <= 0:
                raise ValueError("Contour radius must be positive")
            # Publisher: zero up, clockwise, X right and Y up.
            expected = math.degrees(math.atan2(x, y)) % 360
            if abs((expected - angle + 180) % 360 - 180) > 3:
                raise ValueError("Point coordinates disagree with their angle")
            points[angle] = radius
        self.angles = sorted(points)
        self.radii = [points[a] for a in self.angles]

    def radius_at(self, image_angle):
        if not math.isfinite(image_angle):
            raise ValueError("Angle must be finite")
        angle = image_angle % 360
        if not 30 <= angle <= 330:
            raise ValueError("Missing 0-degree red point: cannot interpolate across 330..30")
        j = bisect_left(self.angles, angle)
        if self.angles[j] == angle:
            return self.radii[j]
        fraction = (angle - self.angles[j - 1]) / 30
        return self.radii[j - 1] + fraction * (self.radii[j] - self.radii[j - 1])

    def feed_at(self, motor_angle, side_image_angle, direction,
                zero_radius_mm=35.0, source_inset_mm=3.0, peel_depth_mm=0.0,
                blade_offset_mm=0.0):
        values = (motor_angle, side_image_angle, zero_radius_mm, source_inset_mm,
                  peel_depth_mm, blade_offset_mm)
        if not all(math.isfinite(v) for v in values) or direction not in (-1, 1):
            raise ValueError("Invalid calibration")
        if zero_radius_mm <= 0 or source_inset_mm < 0 or peel_depth_mm < 0:
            raise ValueError("Invalid radius or depth")
        radius = self.radius_at(side_image_angle + direction * motor_angle)
        # Undo detector inset, then apply requested cutting depth exactly once.
        feed = zero_radius_mm - (radius + source_inset_mm) + peel_depth_mm + blade_offset_mm
        if not 0 <= feed <= 20:
            raise ValueError(f"Unreachable feed {feed:.4f} mm (allowed 0..20)")
        return round(feed, 4)
