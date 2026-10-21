# Existing 30-degree detector integration

## Camera-free mock input

Stop the real detector before using the mock: they publish on the same topic.
After rebuilding and sourcing the workspace, run in separate terminals:

```bash
ros2 run motor_controller yuzu_mock_contour
ros2 run motor_controller yuzu_contour_preview
ros2 topic echo /yuzu/contour_preview
```

Default surface ellipse radii are 30 mm horizontally and 27 mm vertically,
with the detector's 3 mm inward offset. This gives reachable, varying feed
targets under the provisional 35 mm calibration. A circle gives constant feed:

```bash
ros2 run motor_controller yuzu_mock_contour --ros-args -p vertical_radius_mm:=30.0
```

Parameters are read at startup: `horizontal_radius_mm`, `vertical_radius_mm`,
`source_inset_mm`, `publish_rate_hz` (default 10). Use matching source inset
in the preview. Ctrl+C stops publishing; the preview invalidates after 1 s.
The mock and preview do not command motors. They prepare camera-free testing
for the future coordinated motion controller; starting them alone cannot
demonstrate actual motor motion.

This preview consumes `/yuzu/yellow_dot_offsets` (`Float32MultiArray`), exactly
11 triples `[angle_deg, dx_mm, dy_mm]`, angles 30 through 330. Zero is upward,
positive clockwise in the camera image; X right, Y up. The red zero-degree point
is absent. Interpolation is linear in radius between measured angles; the
missing 330-to-30 sector is rejected rather than invented. Interpolation adds
no measured detail. The detector currently uses a fixed 50/232 mm/pixel scale
and a 3 mm radial inset; both require confirmation at the actual camera pose.

After rebuilding the motor_controller package and sourcing its workspace:

```bash
ros2 run motor_controller yuzu_contour_preview
ros2 topic echo /yuzu/contour_preview
```

The node publishes JSON in `std_msgs/String`, with validity, calculated targets,
or a rejection reason. It NEVER publishes motor commands. A one-second missing
input invalidates the preview; the input has no acquisition timestamp, so this
only measures time since receipt, not camera capture age. Multiple detected
fruit cannot be distinguished by this message contract; use one visible fruit.

Provisional mapping: motor 5 zero maps to camera angle 90 (right side), direction
+1. Targets are motor angles -45, -30, 0, 30, 45 degrees: measured knots at
-30/0/30, with interpolated endpoints. Left-side zero can instead be 270.
Camera mounting does not yet establish either mapping. Parameters:
`side_image_angle_deg`, `angle_direction` (+1/-1), `zero_radius_mm` (35),
`source_inset_mm` (3), `peel_depth_mm` (0), and individual
`motor3_offset_mm`/`motor4_offset_mm` (0). Set parameters at startup using
`--ros-args -p name:=value`.

Feed = zero radius - (published radius + source inset) + peel depth + blade offset.
This assumes centered fruit, direct radial slides and shared profile geometry
for both blades. Opposed blades may require separate contour angles; verify
their actual arrangement before motion integration. Feed outside 0..20 mm
rejects the whole preview and is never clipped. The supplied screenshot shows
about 50 mm inset radii, incompatible with a 35 mm zero radius. Confirm scale,
70 mm zero blade gap, and fruit centering before enabling physical following.

This is the profile/preview stage. Continuous multi-axis trajectory execution,
GUI sequence changes, calibrated camera transforms and hardware validation
remain to be implemented. Independent existing axis guards do not themselves
guarantee synchronized contour tracking.
