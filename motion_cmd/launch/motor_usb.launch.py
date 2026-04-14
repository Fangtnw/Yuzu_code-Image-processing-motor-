"""
Launch — USB serial control (motor_controller_node).

Usage:
    ros2 launch motor_controller motor_usb.launch.py

Axis map:
  motor_z              — Z-axis linear (DR28T)    ← USB serial via motor_controller_node
  motor_yuzu_rot       — Yuzu rotation (rotary)   ← stub
  motor_peeler_orbit   — Peeler orbit (rotary)    ← stub
  motor_peeler_advance — Peeler advance (linear)  ← stub (second DR28T, USB)
  motor_conveyor       — Conveyor belt (indexed)  ← stub

Hardware setup:
  1. Connect the AZD-KEP driver to the PC via USB.
  2. Note the COM port (e.g. /dev/ttyACM0 on Linux, COM3 on Windows).
  3. Set COM_PORT below to match.
  4. colcon build && ros2 launch motor_controller motor_usb.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

# ── USB serial linear motors ───────────────────────────────────────────────────
# Add one entry per DR28T connected via USB.
SERIAL_MOTORS = [
    {
        "ns":       "motor_z",
        "motor_id": "motor_z",
        "com_port": "/dev/ttyACM0",   # !! set to your actual COM port
    },
    # Second DR28T (peeler advance) — uncomment when connected:
    # {
    #     "ns":       "motor_peeler_advance",
    #     "motor_id": "motor_peeler_advance",
    #     "com_port": "/dev/ttyACM1",
    # },
]

# ── Rotary / conveyor stubs ────────────────────────────────────────────────────
ROTARY_STUBS = [
    {
        "ns":             "motor_yuzu_rot",
        "motor_id":       "motor_yuzu_rot",
        "spin_up_time_s":  0.3,
        "step_time_s":     0.0,
    },
    {
        "ns":             "motor_peeler_orbit",
        "motor_id":       "motor_peeler_orbit",
        "spin_up_time_s":  0.3,
        "step_time_s":     0.0,
    },
    {
        "ns":             "motor_peeler_advance",
        "motor_id":       "motor_peeler_advance",
        "spin_up_time_s":  0.0,
        "step_time_s":     0.5,
    },
    {
        "ns":             "motor_conveyor",
        "motor_id":       "motor_conveyor",
        "spin_up_time_s":  0.0,
        "step_time_s":     1.5,
    },
]


def generate_launch_description():
    nodes = []

    # ── USB serial motors ──
    for m in SERIAL_MOTORS:
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

    # ── Stubs for motors not yet connected ──
    for m in ROTARY_STUBS:
        # Skip stub if already launched as a real USB motor
        usb_namespaces = {x["ns"] for x in SERIAL_MOTORS}
        if m["ns"] in usb_namespaces:
            continue
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
