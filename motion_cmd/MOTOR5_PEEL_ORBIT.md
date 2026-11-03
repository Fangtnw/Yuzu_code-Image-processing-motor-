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
publishing motor commands. The previous -90° clearance test validated Motor
5 motion and its sign, but it did not validate this new +90° orbit, slide
feeds, the sweep, or the release timing.

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
mean_radius_mm  = (radius_a_mm + radius_b_mm) / 2
slide_target_mm = 35 mm - mean_radius_mm
```

Use the inset contour radius directly to retain the intended 3 mm peel
inset. Do not add the inset again. `motor_controller/yuzu_contour_feed.py`
validates all 72 measured rays, selects the camera's matching 5° points at
each Motor 5 waypoint, averages each blade's mirror rays,
rounds slide targets to their
0.0001 mm count, and rejects targets outside the 0..20 mm guarded stroke.
It does not command motors. The 35 mm distance is a first calibration value;
the two blades can receive separate measured values later.

At Motor 5 +90°, Motor 3 uses detector 0° and Motor 4 uses 180°. During the
counterclockwise sweep, Motor 3's primary ray follows **0° to 100°** and its
mirror ray follows **360° to 260°**. Motor 4's primary ray follows **180° to
280°** and its mirror ray follows **180° to 80°**. The calculated slide
targets are preview values only; automatic movement is not enabled yet.

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
