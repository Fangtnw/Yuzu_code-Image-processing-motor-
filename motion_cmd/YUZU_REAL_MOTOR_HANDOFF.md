> Current integrated physical port wiring: [AXIS_WIRING.md](AXIS_WIRING.md). Motor numbers in this document identify machine roles.

# Yuzu diameter reference and motor sequence

The camera measures the yuzu contour. Motors 3 and 4 use the selected
contour radii for peeling feed. The existing GUI uses fixed Motor 5 angles.
The Step 3 plan moves Motor 5 to +90°, sweeps counterclockwise to -10° with
slide updates every 5°, retracts both slides, then returns Motor 5 to zero.
See
[MOTOR5_PEEL_ORBIT.md](MOTOR5_PEEL_ORBIT.md). The existing GUI still follows
its manual fixed-angle sequence.

## Current machine roles

| Motor | Role |
|---|---|
| 1 | Fruit/camera assembly approach and return |
| 2 | Spin the yuzu during the peel stage |
| 3 and 4 | Move opposing slides to grasp and peel positions |
| 5 | Rotate the peeler mechanism through the peeling orbit |
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
spacing. The new `/yuzu/contour_point_offsets` topic contains 72 measured
triples at 5° spacing, including the red 0° point, for Step 3 feed planning.
Coordinates are relative to the segmentation centroid, X right and
Y up, with zero angle at the top and clockwise positive. The detector's points
are shifted toward the centroid by a configured inset, currently 3 mm. The
horizontal diameter is the sum of the radii at 90° and 270° plus twice the
inset. See `YUZU_CONTOUR_PREVIEW.md` for the mock and preview commands.

`yuzu_contour_profile.py` validates the detector message and extracts this
diameter. `yuzu_contour_preview.py` publishes JSON on `/yuzu/contour_preview`
with `horizontal_diameter_mm`, validity, and `motion_enabled: false`. The
preview never commands a drive. The mock and real detector must not publish
together. The `centdist4.py` detector's assumed 50/410 mm per pixel requires calibration at
the installed camera position. Its message has no fruit ID or capture time;
select and hold a measurement for the fruit being processed before using it
in a sequence.

## Turning diameter into slide positions

The operator reports 35 mm from the yuzu center to each blade at slide
encoder 0 mm. Verify the installed gap and confirm which
way each slide moves, and define distinct clearances for grasp and peel.
Verify fruit centering or provide left/right offsets. A reported 70 mm gap
at both slides' zero is consistent with the 35 mm radial measurement, but
must be checked with the installed blades. A single diameter cannot define
the 5° contour feed distances.

The preliminary slide target is `35 - mean(radius_primary, radius_mirror)`
mm using the inset contour points for each blade. Reject any target outside 0..20 mm;
do not clip it. The 5° orbit and required angle mapping are recorded in
`MOTOR5_PEEL_ORBIT.md`. Confirm the detector-angle direction and blade
geometry before changing the GUI sequence or enabling automatic slide
targets.

## Existing guarded interfaces

The full ROS contract is in `MOTOR_ROS2_INTERFACE.md`. Relevant public command
topics are:

| Topic | Type | Units/range |
|---|---|---|
| `/yuzu/motor2/target_velocity_rpm` | `Float64` | signed rpm, ±416 |
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
