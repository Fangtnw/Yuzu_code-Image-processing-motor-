# ROS 2 interface for an external motor GUI

This is the public interface of `yuzu_peeler.launch.py`. An external GUI can
publish to the **guarded** command topics and subscribe to feedback; it does
not need to access the EtherCAT controller topics. Use the names and ROS 2
message types below exactly. `motor1` through `motor6` are stable machine
identifiers; see the motor/function table in the root `README.md`.

The GUI on the machine PC and an external GUI are independent publishers to
the same command topics. Run only one operator interface with authority to
command motion at a time. The backend does not arbitrate between publishers.
The `/yuzu/motorN/...` topics are the preferred public interface. Simple
targets use `std_msgs/msg/Float64` with the unit in the topic name. Motor 6
indexing uses a repeated request and a status topic.

## Guarded command topics

| Motor | Topic | ROS 2 type | Payload and units |
|---|---|---|---|
| 1 | `/yuzu/motor1/target_position_mm` | `std_msgs/msg/Float64` | `data: absolute_position_mm`; 0–400 mm, aligned to 0.0012 mm |
| 2 | `/yuzu/motor2/target_velocity_rpm` | `std_msgs/msg/Float64` | `data: signed_rpm`; ±332.8 rpm |
| 3 | `/yuzu/motor3/target_position_mm` | `std_msgs/msg/Float64` | `data: absolute_position_mm`; 0–20 mm, 0.0001 mm increments |
| 4 | `/yuzu/motor4/target_position_mm` | `std_msgs/msg/Float64` | `data: absolute_position_mm`; 0–20 mm, 0.0001 mm increments |
| 5 | `/yuzu/motor5/target_position_deg` | `std_msgs/msg/Float64` | `data: signed_angle_deg` relative to the runtime zero; ±180° |
| 5 | `/yuzu/motor5/jog_deg` | `std_msgs/msg/Float64` | `data: relative_step_deg`; guarded jog before setting zero |
| 5 | `/yuzu/motor5/set_zero` | `std_msgs/msg/Empty` | Capture the current position as the runtime zero; confirm the mechanism is stopped first |
| 6 | `/yuzu/motor6/index_request` | `std_msgs/msg/String` | JSON index operation described below |
| 6 | `/yuzu/motor6/stop` | `std_msgs/msg/Empty` | Request a controlled stop; an interrupted index can be resumed |

Motor 2 requires repeated commands while running: the guard's watchdog ramps
to zero after 0.5 s without a new command. Publish at about 10 Hz. The
default acceleration and deceleration are 665.6 rpm/s. To change that profile,
publish `[rpm, acceleration_rpm_s, deceleration_rpm_s]` as
`std_msgs/msg/Float64MultiArray` on the existing
`/motor2_velocity_controller/commands_rpm` topic; then continue refreshing
the simple target. Motor 6 indexing uses its request and heartbeat protocol.

The `*_raw_*_controller/commands` topics are internal `ros2_control` outputs.
Never publish external commands there. Existing
`/motorN_*_controller/...`, `/motor6_conveyor/...`, and `axis1`/`axis2`
command topics remain available for current clients. New clients should use
the `/yuzu/motorN/...` topics above. The existing Motor 6
`/motor6_conveyor/commands_rpm` topic accepts zero for stopping but rejects
nonzero speed in the integrated CSP launch.

## Feedback topics

| Topic | ROS 2 type | Fields used by the operator GUI |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | `motor1_motor2`: `position` is Motor 1 metres; `velocity` is Motor 2 rad/s. `motor4_joint`: `position` is Motor 4 metres; `velocity` is Motor 6 rad/s. |
| `/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | `motor1_motor2` interfaces: `motor3_position` (metres), `motor1_status`, `motor2_status`, `motor3_status`; `motor4_joint` interfaces: `motor5_position` and `motor6_position` (radians), `motor4_status`, `motor5_status`, `motor6_status`. |
| `/yuzu/motor5/zero_position_rad` | `std_msgs/msg/Float64` | Absolute encoder position captured by the Motor 5 guard at `set_zero`, published with transient-local durability for late subscribers. |
| `/yuzu/motor6/index_status` | `std_msgs/msg/String` | JSON status, published every 0.1 s and after each request. |

Match feedback by joint **name** and interface **name**, not by array index.
Convert metres to millimetres and radians/s to rpm (`rpm = rad_s * 30 / pi`)
for display. Motor 5's `motor5_position` is an absolute encoder angle in
radians; subtract `/yuzu/motor5/zero_position_rad` to display its relative
angle. Subscribe to the zero-position topic with transient-local durability
to receive the current zero after joining late. No zero-position message
means that the guard has not accepted a zero capture in this session.

Check message age and all six drive statuswords before enabling motion. The
CiA-402 Operation Enabled test is `(statusword & 0x006F) == 0x0027`. The
included GUI treats feedback older than 2 s as stale; the conveyor index
backend has a tighter feedback requirement. `index_status.fresh` and a recent
status message are both required for conveyor actions. A topic connection or
accepted command does not prove that motion finished.

## Conveyor index protocol

The Motor 6 status JSON includes `session`, `state`, `reference_id`, `index`,
`target_index`, `pitch_mm`, `stopped`, `fresh`, `enabled`, `error_mm`,
`request_id`, `request_ok`, and `request_message`. States include `unreferenced`,
`ready`, `indexing`, `jogging`, `manual`, and `interrupted`. Read a fresh status before
sending a request. Build a unique 1–64 character `id`, copy `session` and
`reference_id` from that status, and set `sent_ns` to the sender's current
ROS clock in nanoseconds. A new request must arrive within 0.5 s of its
timestamp. Both PCs must use
compatible synchronized clocks for the timestamp check.

The payload of `/yuzu/motor6/index_request` is a JSON object in
`std_msgs/msg/String.data`:

| `op` | Additional fields | Meaning |
|---|---|---|
| `reference` | none | Capture a new fixture reference while stopped |
| `index` | `index` (integer), `rpm`, `acceleration`, `deceleration` (rpm/s) | Move to an absolute fixture index relative to the current reference; ±1 from the current index is one step |
| `jog` | `distance_mm`, `rpm`, `acceleration`, `deceleration` | Move a signed distance for alignment |
| `resume` | none | Resume the retained interrupted target |

All requests also require `id`, `session`, `reference_id`, `op`, and `sent_ns`.
Read `request_ok` only when `request_id` matches the sent ID. While an index,
jog, or resume is running, republish the same request JSON with the same ID
about every 100 ms. A repeated accepted request refreshes the 0.5 s guard
watchdog; it does not start a second move. Stopping the repeated command
interrupts the movement. The older `/motor6_conveyor/index_heartbeat` and
`/yuzu/motor6/index_heartbeat` topics still work for existing clients.
Wait for a fresh status with `state: "ready"`,
`stopped: true`, and the expected `target_index` before treating an index as
complete. `pitch_mm` reports the configured nominal fixture spacing; it is
not a measured travel guarantee. See `MOTOR6_INDEXING.md` for its limits.

## Connecting a second laptop

Install a compatible ROS 2 distribution with `std_msgs`, `sensor_msgs`, and
`control_msgs` on the second laptop. Connect both laptops to a network on
which their ROS 2 DDS participants can discover each other. Set the same
`ROS_DOMAIN_ID` and compatible `RMW_IMPLEMENTATION` on both, and verify
discovery with `ros2 topic list` and `ros2 topic info -v` on the second
laptop. The EtherCAT NIC remains dedicated to the motor controllers.

For a read-only first check:

```bash
ros2 topic echo /joint_states
ros2 topic echo /dynamic_joint_states
ros2 topic echo /yuzu/motor6/index_status
ros2 topic info -v /yuzu/motor2/target_velocity_rpm
```

Verify each topic's type and the observed drive status before publishing any
motion command. The physical emergency power cutoff remains necessary.
