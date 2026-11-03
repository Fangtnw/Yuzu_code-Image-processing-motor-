# Motor 5 peeling orbit plan

Motor numbers refer to the existing GUI roles. Motor 5 is the rotary peeler
index on physical Axis 6; Motors 3 and 4 are the two feed slides. The current
Motor 5 backend uses an incremental encoder with an operator-captured runtime
zero and accepts absolute targets from -180° to +180° relative to that zero.

Set zero only while Motor 5 is stopped with the peeler at its intended side
pose. The proposed orbit is:

1. At side pose, capture Motor 5 zero: 0°.
2. Rotate counterclockwise 90° to the clearance position.
3. Feed Motors 3 and 4 into the initial peeling position after Motor 5 has
   settled at -90°. The actual slide positions require geometry calibration.
4. Rotate clockwise through -85°, -80°, ... 0°, +5°, +10°. At every 5°
   target, update the Motor 3/4 feed from the contour profile and wait for
   slide and Motor 5 feedback to settle before the next target.
5. Move Motor 5 from +10° back to its runtime origin at 0°.

This is a 100° clockwise return from the -90° clearance pose. A live 5°
direction test confirmed that the guard's positive signed angle moves this
installed Motor 5 clockwise. `orbit_waypoints(clockwise_sign=1)` therefore
uses -90° for the first counterclockwise move. The planner is in
`motor_controller/yuzu_motor5_orbit_plan.py`. It creates waypoints only; it
does not publish commands or infer slide positions.

On 2026-11-02, with both EtherCAT slaves in OP and Motor 5 Operation Enabled,
the operator confirmed the side pose and clear 90° path. A guarded `-90°`
command was accepted; encoder feedback reached `-16.494869395820135` rad from
the captured origin `-14.924073069025239` rad, a difference of `-π/2` rad.
The return `0°` command restored encoder feedback to the captured origin.
This checked the clearance move and return only. It did not test the 5° peel
sweep, the +10° overshoot, or any Motor 3/4 feed motion. The test used the
working default ROS middleware with Cyclone DDS unset on the motor PC.

The yellow-dot detector currently provides points at 30° increments and no
fruit ID or timestamp. It does not directly supply the 5° profile required
here. Before automatic peeling, the vision and motor teams must agree on
contour interpolation or a higher-resolution contour, angle origin and
direction, and a fresh profile for the fruit being processed. The Motor 3/4
slide geometry, blade clearance, and target limits must also be calibrated.
Each live move must use measured feedback to establish completion; sending a
target is not evidence that the peeler reached it.

The GUI's older operation sequence is manual and describes a return to 0°.
It does not implement the 5° sweep or the 10° overshoot described here.
