# Yuzu contour following: Ubuntu implementation handoff

Date: 2026-10-22. Target: Ubuntu 22.04 / ROS 2 Humble and the existing
`motor_controller` EtherCAT backend. This document is an implementation
requirement, not a claim that coordinated physical peeling already works.

## Goal and machine arrangement

Implement a camera-free demonstration first, then replace mock input with the
real detector without changing the contour message contract.

- Motor 1 positions the fruit/camera assembly for peeling. Hold stationary during
  the first contour demonstration; do not add automatic placement or conveyor motion.
- Motor 2 spins the yuzu about its own axis.
- Motor 5 sweeps the peeling mechanism, carrying motors 3 and 4 around the fruit.
- Motors 3 and 4 feed the blades inward. Increasing either slide position moves
  its blade toward the fruit. Use shared profile feed with individual offsets
  provisionally; verify actual blade arrangement before cutting.
- Provisional motor 5 zero: side of fruit. Provisional sweep: -45 to +45 degrees.
  Motor 5 must move continuously, with smooth coordinated slide motion.
- Current measurements are every 30 degrees. Do not claim 5-degree measured
  resolution. Intermediate targets may be interpolated.

## Existing implementation and limits

Files in `motion_cmd/motor_controller/`:

| File | Current role |
|---|---|
| `yuzu_mock_contour.py` | Synthetic ellipse, same topic/type/layout as detector |
| `yuzu_contour_profile.py` | Validate 11 rays, interpolate radii, compute bounded feed |
| `yuzu_contour_preview.py` | Publish calculated targets as JSON; NO motor commands |
| `azd3a_motor3_position_guard.py` | Independent 200 Hz position ramp |
| `azd3a_motor4_position_guard.py` | Independent 200 Hz position ramp |
| `azd3a_motor5_position_guard.py` | Independent 200 Hz position ramp and runtime zero |
| `yuzu_peeler_gui.py` | Manual control and existing step sequence |

Entry points `yuzu_mock_contour` and `yuzu_contour_preview` are registered in
`motion_cmd/setup.py`. Eight contour/mock unit tests passed offline on Windows;
syntax checks passed. ROS execution and hardware tracking have not been tested.

The preview currently evaluates motor angles -45, -30, 0, 30, 45. Its default
side mapping is image angle 90 degrees, direction +1. These are placeholders.
Its timeout is time since message receipt, not camera acquisition age.

The existing GUI step 4.3 sends motor 5 back to zero and immediately stops
motor 2. Replace that behavior for contour mode: keep motor 2 running through
the sweep, and stop only after verified sweep completion and the planned
blade withdrawal. Do not assume publishing a target means motion completed.

## Input contract

Topic: `/yuzu/yellow_dot_offsets`

Type: `std_msgs/msg/Float32MultiArray`

Payload: 33 values, 11 triples `[angle_deg, dx_mm, dy_mm]`, angles
30, 60, 90, ..., 330. The red zero-degree point is NOT published.

Layout dimensions: `yellow_dots`, size 11, stride 33; `angle_dx_dy_mm`,
size 3, stride 3. The existing parser validates data rather than relying on layout.

Coordinates are relative to the segmentation centroid: X right, Y up;
angle zero at the top and increasing clockwise. Published points are already
moved 3 mm toward the centroid. The detector currently assumes 50/232 mm per
pixel, which requires calibration at the actual fruit location.

Use exactly one input publisher: mock OR camera. The message has no fruit ID,
sequence number, capture timestamp or validity flag. For the initial demo,
use one fruit and freeze a validated profile before motion. A stamped message
and explicit acquisition/selection protocol are required for reliable live updates.

## Geometry and equations

For each point:

```text
r_published = sqrt(dx_mm^2 + dy_mm^2)
r(theta) = r_a + (theta - theta_a)/(theta_b - theta_a) * (r_b - r_a)
```

Interpolate only between available neighboring rays. Reject requests in the
missing sector around zero (330 -> 360 -> 30); do not silently extrapolate.

Provisional angle mapping:

```text
theta_image = side_image_angle_deg + angle_direction * motor5_angle_deg
```

This is only valid for sufficiently aligned viewing/sweep planes and compatible
origins. The detected centroid is not necessarily the motor 5 pivot. Calibrate
scale, camera pose, pivot offset, zero and sign before real image-based cutting.
A planar transform cannot recover arbitrary unseen 3D fruit geometry.

Provisional radial feed equation, independently for each blade:

```text
feed_i_mm = zero_radius_mm
            - (r_published(theta_image) + source_inset_mm)
            + peel_depth_mm + blade_offset_i_mm
```

Defaults: zero radius 35 mm, source inset 3 mm, requested peel depth 0 mm,
blade offsets 0. Undo the source inset before applying desired depth; never
apply the 3 mm cut twice. This is radial depth, not surface-normal thickness.
Equation assumes direct radial slide travel and a centered fruit/pivot.
Different contact locations may require different contour angles for each blade.

Hardware report: zero-position gap between blades is 70 mm, provisionally
35 mm each from center (not 3.5 mm). Screenshot published radius is about
50 mm, incompatible with 35 mm zero clearance. Resolve scale/gap/centering
before cutting. Never hide this conflict by clipping negative feed to zero.

## Existing ROS interfaces to preserve

Full contract: `MOTOR_ROS2_INTERFACE.md` in this directory.

| Command topic | Type | Units / bounds |
|---|---|---|
| `/yuzu/motor2/target_velocity_rpm` | `std_msgs/msg/Float64` | signed rpm, +/-332.8 |
| `/yuzu/motor3/target_position_mm` | `std_msgs/msg/Float64` | absolute mm, 0..20, increment 0.0001 |
| `/yuzu/motor4/target_position_mm` | `std_msgs/msg/Float64` | absolute mm, 0..20, increment 0.0001 |
| `/yuzu/motor5/target_position_deg` | `std_msgs/msg/Float64` | runtime-relative degrees, +/-180 |
| `/yuzu/motor5/set_zero` | `std_msgs/msg/Empty` | capture zero while stopped |

Motor 2 needs repeated commands about 10 Hz; its watchdog ramps to zero after
0.5 seconds without commands. Its default ramp is 665.6 rpm/s. This is an
existing maximum/default, not a recommended first-test speed.

Slides 3/4: velocity cap 32 mm/s and acceleration cap 64 mm/s^2.
Motor 5: velocity cap 120 rpm and acceleration cap 240 rpm/s. Use configurable
lower demonstration limits; do not use maximum limits as demo defaults.

Feedback:

- `/joint_states`: `motor4_joint.position` = motor 4 metres;
  `motor1_motor2.velocity` = motor 2 rad/s.
- `/dynamic_joint_states`: joint `motor1_motor2`, interface `motor3_position`
  = metres; joint `motor4_joint`, interface `motor5_position` = absolute radians.
  Drive status interfaces are documented in the public contract.
- `/yuzu/motor5/zero_position_rad`: `Float64`, transient-local durability.
  Relative motor 5 angle = degrees(absolute feedback - captured zero).

Match names, not array indices. Require fresh feedback and Operation Enabled
status `(statusword & 0x006F) == 0x0027` for all six drives, consistent with the
existing GUI. Topic presence alone is insufficient.

## Required coordinated controller work

1. Add a dedicated contour execution node/state machine, separate from the
   synthetic publisher and preview. Default to no motion until explicit start.
2. Provide configurable input source, calibration, sweep bounds/direction,
   slide offsets, depth, motor 2 rpm, sweep speed, tracking tolerances,
   feedback timeout and endpoint settling time. Start/stop/status must be explicit.
3. Capture and validate a complete profile; validate the whole interpolated
   trajectory against position, speed and acceleration bounds before starting.
   Do not execute a partial profile when one target fails.
4. Sequence: ready -> blades clear -> position sweep start -> establish initial
   feed/spin under a defined approach plan -> coordinated sweep -> planned
   blade withdrawal -> stop spin -> complete. Wait on feedback at transitions.
5. Interpolate feed continuously between 30-degree knots. A starting position
   or feed approach is a separate move, not an instantaneous jump. Use a common
   trajectory progress/time base and acceleration-limited transitions.
6. Independent public position guards currently reprofile each target. Simply
   publishing simultaneous setpoints does NOT guarantee synchronization.
   Choose and document an integrated guarded trajectory mode or compatible
   shared planner; preserve manual mode and ensure exclusive command ownership.
   External nodes must not bypass guards by publishing raw controller topics.
7. Monitor measured motor 5 angle and both measured slide positions against the
   planned contour. Slow/pause the sweep or abort on excess tracking error;
   do not let motor 5 continue while slide feed lags.
8. Implement backend timeouts/controlled stop for motors 3/4/5. Existing guards
   retain their last position target; killing the new publisher alone will not
   stop an ongoing move. Define the stop behavior before testing real motion.
9. Abort on feedback loss, disabled/faulted drive, invalid start profile, zero
   reset during execution, limits, tracking failure, operator stop or shutdown.
   Define coordinated deceleration and retraction policy: do not blindly retract
   with unknown feedback or continue cutting after an error. No automatic restart.
10. Isolate the operator GUI from execution command ownership. GUI periodic
    motor 2 refreshes can conflict even without manual button presses; merely
    asking the operator not to click is insufficient. Add a backend-only launch
    or explicit GUI/execution mode arbitration. Run one EtherCAT backend only.

Use a frozen profile for the first demo. If future live updates are allowed,
define stale-input behavior and bounded profile changes. Do not apply abrupt
frame-to-frame segmentation noise to blades. Fruit rotation changes its shape;
a single silhouette is only an approximate rotational profile.

## Ubuntu preparation and camera-free verification

Use the existing setup instructions for EtherCAT/ROS installation. These commands
assume an already configured `~/yuzu_ws` workspace linked to this repository.
Stop any active control session before rebuilding.

```bash
source /opt/ros/humble/setup.bash
cd ~/yuzu_ws
colcon build --packages-up-to motor_controller --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

In separate sourced terminals (no powered motor backend needed):

```bash
ros2 run motor_controller yuzu_mock_contour
ros2 run motor_controller yuzu_contour_preview
ros2 topic echo /yuzu/contour_preview
```

Expect `valid: true`, `motion_enabled: false`, and varying proposed feed targets.
Default mock surface radii are 30 mm horizontal and 27 mm vertical. A circle:

```bash
ros2 run motor_controller yuzu_mock_contour --ros-args -p vertical_radius_mm:=30.0
```

Expect both slides to have 5 mm calculated feed at all preview angles with
default calibration. Stop the first mock before starting another. Stopping
input should produce invalid preview after 1 second. A 50 mm mock circle
should be rejected as unreachable under the 35 mm zero calibration.

From the repository root:

```bash
python3 -m unittest discover -s tests -p 'test_yuzu_*.py'
python3 -m unittest discover -s tests -p 'test_*.py'
python3 -m py_compile motion_cmd/motor_controller/*.py motion_cmd/launch/*.py
```

Do not interpret successful preview/tests as physical motion verification.
The coordinated controller executable and its start command do not exist yet;
add tested commands here after implementing them.

## Demonstration acceptance stages

1. Offline mock: verify circle/ellipse feed, missing points, NaN/duplicate inputs,
   limit rejection, direction mapping, timing, stop and timeout behavior.
2. Hardware with blades clear: verify zero/sign, low-speed sweep, measured
   tracking and coordinated stop without fruit contact. Keep the established
   physical emergency cutoff accessible.
3. Known-size calibration object with rotation stopped: establish contact
   references and actual slide-to-radius conversion separately for both blades.
4. Real fruit: verify camera scale/pivot mapping and clearance, then demonstrate
   a shallow configurable cut at validated low speeds. Record actual trajectories.

Required logs: state, input source/profile ID, calibrated parameters, commanded
and measured motor 5 angle, slide targets/positions, motor 2 command/feedback,
tracking errors, statusword readiness and fault/stop reason with timestamps.

Before declaring success, demonstrate endpoint settling, no motion on startup,
stop during sweep, feedback loss, source loss under the selected policy,
out-of-range profile rejection and no restart after a fault. Preserve unrelated
uncommitted company-review work; do not reset/clean the repository.
