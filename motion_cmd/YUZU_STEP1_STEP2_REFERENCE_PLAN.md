# Yuzu Steps 1 and 2: Motor 1 camera reference

Updated 2026-11-03. Motor numbers retain the current GUI machine roles and
backend wiring in [AXIS_WIRING.md](AXIS_WIRING.md). The motor-control team
subscribes to the detector on the separate camera PC. No motor controller
launch or automatic motion is required to preview the target.

## Step 1: detect at home and pierce

Motor 1 stands by at its measured home encoder position. The camera detects
the red 0° top point and publishes `/yuzu/y_offset_mm` as
`std_msgs/msg/Float32`; see [MOTOR1_VISION_HANDOFF.md](MOTOR1_VISION_HANDOFF.md)
for its sign and inset. The operator measured encoder position **204 mm**
when the holder tip reference reaches the camera frame-center plane. This is
the `q_frame_mm` reference used for the absolute Motor 1 target.

Motor 1 increases downward. With the detector's signed offset, the target is:

```text
q_insert_mm = q_frame_mm - y_offset_mm
              + (pierce_depth_mm - detector_inset_mm) + allowance_mm
```

The current detector red point is 3 mm inside the original yuzu contour.
Default pierce depth is 3 mm and extra allowance is 0 mm, so neither 3 mm is
added twice. A negative detector offset increases the target and moves Motor 1
farther downward. With default depths and zero allowance, the target is
`204 - y_offset_mm`, or `204 + camera_depth_mm` when
`camera_depth_mm = -y_offset_mm`. The former `260 - 5 - vertical_height - 3` floor-reference
calculation is historical and is not used for this camera-based Step 1.

Preview using one newly published detector value:

```bash
ros2 run motor_controller yuzu_motor1_reference --offset-from-detector
```

Or preview a signed manual offset with `--y-offset-mm VALUE`. The script
prints the input and both Step 1/2 targets without commanding hardware.
Only after checking the calibration and a clear path, add `--step insert
--execute`. The CLI requires fresh Motor 1 position and Operation Enabled
status, a current position within 0.1 mm of the specified home pose, and a
guarded target inside 0..400 mm. Confirm movement from feedback.

## Step 2: lift

The Step 2 target is `q_lift_mm = q_insert_mm - lift_mm`, where `lift_mm` must
be explicitly chosen in the documented 100..150 mm range for execution.
The CLI requires Motor 1 feedback at the calculated insertion target before
it sends a lift. Reuse the same captured detector offset and calibration
values for Steps 1 and 2; a fresh measurement from another fruit must not
change the insertion target between those steps.

## Remaining calibration and integration

- Recheck the 204 mm frame-center encoder reference, the exact home encoder
  position, and the physical holder tip reference after mechanical changes.
  Confirm the camera scale at the installed pose.
- Verify the 3 mm detector contour inset and the desired physical piercing
  depth at the installed hardware. Extra allowance defaults to zero.
- Associate a detector offset with the current fruit. The current `Float32`
  message has no fruit ID or capture timestamp.
- Maintain the camera-acquisition timing and subsequent contour processing
  handoff separately. Motor 3/4 depth and Motor 5 orbit planning are recorded
  in their own documents.

The earlier 260 mm holder-to-conveyor-floor observation and approximately
5 mm fruit-bottom gap remain historical measurements; they do not define the
new camera-based Motor 1 target.
