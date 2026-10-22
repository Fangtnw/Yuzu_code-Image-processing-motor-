# Yuzu diameter reference and fixed motor sequence

Updated 2026-10-22 for the corrected machine concept. The camera measures the
yuzu's **horizontal diameter**. Motors 3 and 4 use that measurement as a
reference for grasping and peeling. Motor 5 rotates through a **fixed sequence**;
its target angle is independent of the measured contour shape. The earlier
angle-by-angle slide-feed and contour-following sweep plan is superseded.

## Current machine roles

| Motor | Role |
|---|---|
| 1 | Fruit/camera assembly approach and return |
| 2 | Spin the yuzu during the peel stage |
| 3 and 4 | Move opposing slides to grasp and peel positions |
| 5 | Rotate peeler mechanism through fixed targets |
| 6 | Index the next fruit fixture |

The existing GUI already has a manually advanced sequence: Motor 6 index,
Motor 1 approach, Motor 5 to +90°, Motor 2 spin, Motors 3/4 feed in, Motor 5
to 0° with a Motor 2 stop command, Motors 3/4 grip, Motor 1 return, Motors
3/4 home, and cycle wait. This describes current software behavior, not a
validated automatic peeling cycle. The fixed Motor 5 angles and the exact
meaning/order of slide feed and grip positions need confirmation for the
physical setup. The GUI's Motor 3/4 positions remain manually entered.

## Camera input and diameter

`/yuzu/yellow_dot_offsets` is a `std_msgs/msg/Float32MultiArray` of 33 values:
11 triples `[angle_deg, dx_mm, dy_mm]` for angles 30° through 330° at 30°
spacing. Coordinates are relative to the segmentation centroid, X right and
Y up, with zero angle at the top and clockwise positive. The detector's points
are shifted toward the centroid by a configured inset, currently 3 mm. The
horizontal diameter is the sum of the radii at 90° and 270° plus twice the
inset. See `YUZU_CONTOUR_PREVIEW.md` for the mock and preview commands.

`yuzu_contour_profile.py` validates the detector message and extracts this
diameter. `yuzu_contour_preview.py` publishes JSON on `/yuzu/contour_preview`
with `horizontal_diameter_mm`, validity, and `motion_enabled: false`. The
preview never commands a drive. The mock and real detector must not publish
together. The detector's assumed 50/232 mm per pixel requires calibration at
the installed camera position. Its message has no fruit ID or capture time;
select and hold a measurement for the fruit being processed before using it
in a sequence.

## Turning diameter into slide positions

Before software can calculate Motor 3/4 absolute targets, measure the actual
gap between blade reference surfaces at known slide positions, confirm which
way each slide moves, and define distinct clearances for grasp and peel.
Verify fruit centering or provide left/right offsets. A reported 70 mm gap
at both slides' zero is provisional and must be checked with the installed
blades. The camera diameter cannot by itself determine a safe feed distance.

Once those quantities are calibrated, derive each slide target from the
diameter and its own blade geometry, reject any target outside 0..20 mm,
and require a fresh measurement for the current fruit. Do not clip an
unreachable target. Keep Motor 5 angle targets independent of the camera
diameter. Await operator confirmation of the intended fixed angles/order
before changing the GUI sequence or enabling automatic slide targets.

## Existing guarded interfaces

The full ROS contract is in `MOTOR_ROS2_INTERFACE.md`. Relevant public command
topics are:

| Topic | Type | Units/range |
|---|---|---|
| `/yuzu/motor2/target_velocity_rpm` | `Float64` | signed rpm, ±332.8 |
| `/yuzu/motor3/target_position_mm` | `Float64` | absolute mm, 0..20 |
| `/yuzu/motor4/target_position_mm` | `Float64` | absolute mm, 0..20 |
| `/yuzu/motor5/target_position_deg` | `Float64` | runtime-relative degrees, ±180 |
| `/yuzu/motor5/set_zero` | `Empty` | capture stopped reference |

Motor 2's velocity watchdog needs periodic commands while spinning. The
Motor 3/4/5 position guards retain their last target. Use feedback, rather
than command publication, to decide when a stage has completed. Fresh
position feedback and Operation Enabled status on the drives are required
before a powered test. Keep the physical stop accessible.

## Next commissioning check

1. Run the real detector and diameter preview with one fruit. Verify the
   horizontal diameter against caliper measurements at the installed camera
   pose; confirm the inset applied by the real detector.
2. Measure blade gap and slide direction at known Motor 3/4 positions. Record
   grasp clearance, peel depth, and any left/right offset separately.
3. Confirm the fixed Motor 5 target sequence and zero pose. Verify each move
   with the blades clear and measured Motor 5 feedback.
4. Calculate Motor 3/4 targets only after the geometry is known, validate
   each against the guarded 0..20 mm range, and stage a no-contact run before
   touching fruit.
