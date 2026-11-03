# Motor 3/4 contour feed and Motor 5 peeling orbit

Motor numbers keep the current GUI roles. Motor 5 is the rotary peeler index
on physical Axis 6. A live 5° direction test established that a **positive**
signed Motor 5 target rotates physically **clockwise**. At Motor 5's runtime
zero, the peeler is at the side-of-yuzu origin pose.

## Operator-confirmed order

1. With Motors 3/4 at 0 mm, move Motor 5 clockwise to **+90° standby**.
2. At +90°, Motor 3's blade uses the detector's red **0° top point**; Motor
   4's blade uses the **180° bottom point**. Feed both slides to their calculated
   starting targets.
3. Sweep Motor 5 **100° counterclockwise**, +90°, +85°, ... 0°, -5°, -10°.
   At each 5° step, update Motors 3 and 4 from the corresponding contour
   radii and wait for feedback before the next step.
4. At **-10°**, retract both slides to **0 mm** to release the yuzu. Confirm
   retraction from feedback.
5. Move Motor 5 **10° clockwise** back to its runtime **0° origin**.

`motor_controller/yuzu_motor5_orbit_plan.py` generates this order without

## Camera points and slide targets

`image_proc/yuzu_detection_curve_centdist4.py` now publishes
`/yuzu/contour_point_offsets` as `std_msgs/msg/Float32MultiArray`: 216 values,
72 measured triples `[angle_deg, dx_mm, dy_mm]` for 0°, 5°, ..., 355°. Coordinates
are relative to the yuzu centroid; X points right, Y points up, angles
increase clockwise from the top. The points are on the contour already
shifted 3 mm inward. The existing 11-point `/yuzu/yellow_dot_offsets` and
`/yuzu/y_offset_mm` topics remain unchanged. The new publisher must be
deployed and restarted on the vision PC before it appears in the motor PC's
ROS graph.

At slide encoder **0 mm**, each blade reference is **35 mm** from the yuzu
center. The yuzu rotates about itself during peeling. The operator selected a
**mirror pair** of contour rays for each blade at each Motor 5 step. For
example, Motor 3 averages 30° with 330°; Motor 4 averages 210° with 150°.
For each blade:

```text
radius_a_mm     = hypot(dx_a_mm, dy_a_mm)
radius_b_mm     = hypot(dx_b_mm, dy_b_mm)
mean_inset_radius_mm = (radius_a_mm + radius_b_mm) / 2
outer_radius_mm = mean_inset_radius_mm + source_inset_mm
slide_target_mm = 35 mm - outer_radius_mm + peel_depth_mm
```

The operator selected the **outer fruit surface** with **0 mm blade
penetration** for the current preview. `source_inset_mm` defaults to 3 mm
because the camera publishes an inward-shifted contour; `peel_depth_mm`
defaults to 0 mm. Restore that inset once to estimate the original outer
radius. `motor_controller/yuzu_contour_feed.py`
validates all 72 measured rays, selects the camera's matching 5° points at
each Motor 5 waypoint, averages each blade's mirror rays,
rounds slide targets to their
0.0001 mm count, and rejects targets outside the 0..20 mm guarded stroke.
It does not command motors. The 35 mm distance is a first calibration value;
the two blades can receive separate measured values later.

A live 5° camera sample previously gave Motor 4 a mean inset radius near
13.9 mm at Motor 5 +90°. Using the inset directly produced a 21.1 mm slide
target outside the 20 mm stroke. Restoring the 3 mm source inset gives an
estimated 18.1 mm outer-surface target. The correction is an approximation
because the detector shifts contour vertices toward the centroid before
casting rays; confirm blade contact against the actual fruit and adjust the
camera or blade calibration if needed. The preview continues to reject any
calculated target outside the guarded stroke.

On 2026-11-03, a fresh 72-point live sample passed the corrected no-motion
preview at all 21 sweep waypoints. The printed Motor 3 targets ranged about
14.18..15.89 mm and Motor 4 targets about 12.78..17.85 mm. These are one
fruit's estimated outer-surface positions, not a powered blade-contact test.

At Motor 5 +90°, Motor 3 uses detector 0° and Motor 4 uses 180°. During the
counterclockwise sweep, Motor 3's primary ray follows **0° to 100°** and its
mirror ray follows **360° to 260°**. Motor 4's primary ray follows **180° to
280°** and its mirror ray follows **180° to 80°**. The calculated slide
targets are used by the guarded Step 3 coordinator after a complete contour
and motor start state have been checked.

The full contour topic now measures every 5°; the legacy yellow and diameter
topics still use 30° points. No radius interpolation is used for 5° feed
targets. Before powered contour peeling, compare the preview with the
installed geometry and blade direction. A complete profile must be captured for the
fruit being peeled; the current array has no fruit ID or capture timestamp.

After the vision PC runs the updated `centdist4.py`, inspect one profile on
the motor PC with:

```bash
ros2 topic list -t --no-daemon | grep /yuzu/contour_point_offsets
ros2 run motor_controller yuzu_contour_feed_preview
```

The preview prints 21 rows: the +90° starting feed and each 5° sweep target
through -10°. It sends no commands. It validates every slide target against
0..20 mm before printing the table. At -10°, the subsequent release is a
separate command setting Motors 3/4 to 0 mm before Motor 5 returns to 0°.

## Motor 5-only orbit check (2026-11-03)

With both slides retracted to 0 mm, the operator placed Motor 5 at its side
origin, captured runtime zero, and confirmed a clear path. Both EtherCAT
slaves were OP and Motor 5 was Operation Enabled. Guarded absolute targets
`+90°`, `-10°`, and `0°` were sent one at a time. Encoder feedback moved from
`-18.07094359827306` rad at origin to `-16.500147271478166` rad at +90°,
then `-18.24549048610651` rad at about -10°, and returned to the exact
origin reading. Motors 3/4 remained at 0 mm. This verifies the Motor 5 path
only; no contour feed or blade contact was commanded.

## Coordinated Step 3 command

Run `ros2 run motor_controller yuzu_step3_sequence` to print and validate the
24-action plan without moving motors. Run
`ros2 run motor_controller yuzu_step3_sequence --execute` only after checking
blade clearance and the intended fruit. The command freezes one 72-ray
contour, validates all 21 feed targets before motion, requires fresh enabled
feedback for Motors 3/4/5, both slides at 0 mm, Motor 5 at its operator-set
runtime zero, and all three guarded command subscribers. It moves Motor 5
to +90°, feeds the slides, then commands one continuous Motor 5 sweep to
-10°. During the sweep it interpolates the prevalidated 5° slide targets
from measured Motor 5 angle and updates both slides at 20 Hz. The Motor 5
guard limits sweep speed to 10°/s by default; `--sweep-speed-deg-s` accepts
1..30°/s. The sequence checks that neither slide trails its contour target
by more than 1 mm. At -10° it retracts both slides, then returns Motor 5
to 0°. Setup, release, and return still wait for measured settling. A stale
status or timeout stops the sequence
at the last commanded targets; the operator must inspect the machine before
recovering. The contour is frozen for one run and the camera topic has no
fruit identifier or timestamp, so check that its preview matches the fruit
being peeled.

The updated Motor 5 guard must be restarted before using `--execute`;
the sequence requires its guarded speed-limit subscriber.
