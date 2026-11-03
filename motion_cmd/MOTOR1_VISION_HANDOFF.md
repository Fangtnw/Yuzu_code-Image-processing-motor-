# Motor 1 camera offset input

Motor 1 stays at its measured home position while the camera detects the
yuzu top. The vision program `image_proc/yuzu_detection_curve_centdist4.py`
publishes `/yuzu/y_offset_mm` as `std_msgs/msg/Float32`. The value is the
vertical offset from the camera frame center to the red 0° point. A positive
value means the point is above the frame center; a negative value means it is
below. The red point is calculated from the contour shifted 3 mm inward.

The operator measured Motor 1 encoder position **204 mm** when the holder tip
reference is at the camera frame-center plane. Call this `q_frame_mm`. Motor 1
stands by at its home position for detection, then moves to an absolute
encoder target. The positive Motor 1 direction is downward. Define a
positive-down camera depth as `depth_mm = -y_offset_mm`. The target formula is:

```text
q_insert = q_frame + depth + (pierce_depth - detector_inset) + allowance
         = 204 - y_offset + allowance    (with both depth and inset at 3 mm)
```

The defaults are `pierce_depth = 3 mm`, `detector_inset = 3 mm`, and
`allowance = 0 mm`. Thus the two 3 mm values cancel and the normal target is
`204 - y_offset`. A negative offset increases the downward target, as
confirmed by the operator. `--allowance-mm` adds extra downward travel when
explicitly set; it must be nonnegative. The calculation does not reuse the
old 260 mm conveyor-floor reference or the fruit's full vertical height.

Preview without motion using the 204 mm reference:

```bash
ros2 run motor_controller yuzu_motor1_reference --offset-from-detector
```

For a manual input, replace `--offset-from-detector` with
`--y-offset-mm -10` (giving a 214 mm target at zero extra allowance).
Override the frame reference with `--frame-center-mm VALUE` only if it is
remeasured. The script waits up to five seconds for a fresh detector
value when using the topic. It rejects targets outside the guarded 0..400 mm
stroke. To command Step 1, Motor 1 must have fresh Operation Enabled feedback
at the measured home position; add `--step insert --execute` only after
checking the preview. Step 2 still requires an explicit 100..150 mm lift and
feedback at the calculated insertion target.

The camera scale in `centdist4.py` is currently hard-coded to `50/410`
mm/pixel. Verify it at the installed pose. The `Float32` topic has no fruit ID
or capture timestamp, so a future automatic cycle must associate each offset
with the fruit currently at Motor 1. The vision detector is owned and run by
the separate camera team; the motor side only subscribes to its topic.
