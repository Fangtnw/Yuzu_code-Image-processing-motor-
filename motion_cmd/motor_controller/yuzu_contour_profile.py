"""Validate the detector's 11-ray message and read its horizontal diameter."""

import math


class ContourProfile:
    def __init__(self, data):
        if len(data) != 33:
            raise ValueError("Expected 11 yellow points: [angle, dx_mm, dy_mm] x 11")
        points = {}
        for i in range(0, len(data), 3):
            angle, x, y = map(float, data[i:i + 3])
            if not all(math.isfinite(v) for v in (angle, x, y)):
                raise ValueError("Detector message contains non-finite values")
            if angle not in range(30, 360, 30) or angle in points:
                raise ValueError("Expected unique angles 30..330 at 30-degree spacing")
            radius = math.hypot(x, y)
            if radius <= 0:
                raise ValueError("Detector radius must be positive")
            # Publisher: zero up, clockwise, X right and Y up.
            expected = math.degrees(math.atan2(x, y)) % 360
            if abs((expected - angle + 180) % 360 - 180) > 3:
                raise ValueError("Point coordinates disagree with their angle")
            points[angle] = radius
        self.radii_mm = points

    def horizontal_diameter_mm(self, source_inset_mm=3.0):
        """Recover the diameter from the inward-shifted 90° and 270° rays."""
        if not math.isfinite(source_inset_mm) or source_inset_mm < 0:
            raise ValueError("Source inset must be finite and nonnegative")
        return round(self.radii_mm[90] + self.radii_mm[270] +
                     2.0 * source_inset_mm, 4)
