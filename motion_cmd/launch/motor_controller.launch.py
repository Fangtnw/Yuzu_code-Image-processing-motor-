"""
Launch file for the 5-DOF Yuzu Peeler motor system.

Axis map:
  motor_z              — Z-axis linear (DR28T)          ← connected now
  motor_yuzu_rot       — Yuzu rotation spindle (rotary) ← stub until hardware arrives
  motor_peeler_orbit   — Peeler arm orbit (rotary)      ← stub
  motor_peeler_advance — Peeler radial advance (linear) ← stub (second DR28T)
  motor_conveyor       — Conveyor belt (indexed)        ← stub

To activate a real motor:
  1. Move it from ROTARY_STUBS / ADVANCE_STUB to LINEAR_MOTORS (if DR28T)
     or replace its stub entry with the real driver node.
  2. Set its com_port to the correct serial device.
  3. colcon build && ros2 launch motor_controller motor_controller.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

# ── Linear motors (DR28T, MEXE02 serial protocol) ─────────────────────────────
LINEAR_MOTORS = [
    {
        "ns":       "motor_z",
        "motor_id": "motor_z",
        "com_port": "/dev/ttyACM0",   # Z-axis — connected
    },
    # Uncomment when second DR28T (peeler advance) arrives:
    # {
    #     "ns":       "motor_peeler_advance",
    #     "motor_id": "motor_peeler_advance",
    #     "com_port": "/dev/ttyACM1",
    # },
]

# ── Rotary / conveyor stubs (replace with real drivers when hardware arrives) ─
ROTARY_STUBS = [
    {
        "ns":            "motor_yuzu_rot",
        "motor_id":      "motor_yuzu_rot",
        "spin_up_time_s": 0.3,
        "step_time_s":    0.0,
    },
    {
        "ns":            "motor_peeler_orbit",
        "motor_id":      "motor_peeler_orbit",
        "spin_up_time_s": 0.3,
        "step_time_s":    0.0,
    },
    {
        "ns":            "motor_peeler_advance",
        "motor_id":      "motor_peeler_advance",
        "spin_up_time_s": 0.0,
        "step_time_s":    0.5,
    },
    {
        "ns":            "motor_conveyor",
        "motor_id":      "motor_conveyor",
        "spin_up_time_s": 0.0,
        "step_time_s":    1.5,
    },
]


def generate_launch_description():
    nodes = []

    # ── Real linear motor nodes ──
    for m in LINEAR_MOTORS:
        nodes.append(Node(
            package="motor_controller",
            executable="motor_controller_node",
            name="motor_controller_node",
            namespace=m["ns"],
            parameters=[{
                "com_port": m["com_port"],
                "motor_id": m["motor_id"],
            }],
            output="screen",
        ))

    # ── Stub nodes for motors not yet connected ──
    for m in ROTARY_STUBS:
        nodes.append(Node(
            package="motor_controller",
            executable="rotary_motor_stub_node",
            name="rotary_motor_stub_node",
            namespace=m["ns"],
            parameters=[{
                "motor_id":       m["motor_id"],
                "spin_up_time_s": m.get("spin_up_time_s", 0.3),
                "step_time_s":    m.get("step_time_s", 1.5),
            }],
            output="screen",
        ))

    # ── Harvest sequence orchestrator ──
    nodes.append(Node(
        package="motor_controller",
        executable="harvest_sequence_node",
        name="harvest_sequence",
        output="screen",
    ))

    return LaunchDescription(nodes)
