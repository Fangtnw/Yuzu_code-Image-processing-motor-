"""
Launch — USB serial control (motor_controller_node).

Usage:
    ros2 launch motor_controller motor_usb.launch.py

Axis map:
  motor_z            — Z-axis linear (DR28T)       ← USB serial via motor_controller_node
  motor_yuzu_rot     — Yuzu rotation (rotary)      ← stub  (Type 2)
  motor_peeler_3     — Peeler feed top (linear)    ← stub  (Type 1)
  motor_peeler_4     — Peeler feed bottom (linear)  ← stub  (Type 1)
  motor_peeler_orbit — Peeler orbit (rotary)       ← stub  (Type 2)
  motor_conveyor     — Conveyor (linear, Type 1)   ← stub  (Type 1)

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
    # Peeler feed motors — uncomment when connected:
    # {
    #     "ns":       "motor_peeler_3",
    #     "motor_id": "motor_peeler_3",
    #     "com_port": "/dev/ttyACM1",
    # },
    # {
    #     "ns":       "motor_peeler_4",
    #     "motor_id": "motor_peeler_4",
    #     "com_port": "/dev/ttyACM2",
    # },
]

# ── Motor stubs (for all motors not yet connected to real hardware) ────────────
#
#   Type 2 (rotary) stubs: motor_spin [Float32MultiArray: rpm, accel, decel]
#   Type 1 (linear) stubs: motor_cmd  [Float32MultiArray: pos_mm, speed, ...]
#                           motor_home [Empty]
MOTOR_STUBS = [
    # ── Rotary (Type 2) ──
    {
        "ns":            "motor_yuzu_rot",
        "motor_id":      "motor_yuzu_rot",
        "spin_up_time_s": 0.3,
    },
    {
        "ns":            "motor_peeler_orbit",
        "motor_id":      "motor_peeler_orbit",
        "spin_up_time_s": 0.3,
    },
    # ── Linear (Type 1) ──
    {
        "ns":          "motor_peeler_3",
        "motor_id":    "motor_peeler_3",
        "move_time_s":  2.0,
    },
    {
        "ns":          "motor_peeler_4",
        "motor_id":    "motor_peeler_4",
        "move_time_s":  2.0,
    },
    {
        "ns":          "motor_conveyor",
        "motor_id":    "motor_conveyor",
        "move_time_s":  2.0,
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
    usb_namespaces = {m["ns"] for m in SERIAL_MOTORS}
    for m in MOTOR_STUBS:
        if m["ns"] in usb_namespaces:
            continue
        nodes.append(Node(
            package="motor_controller",
            executable="motor_stub_node",
            name="motor_stub_node",
            namespace=m["ns"],
            parameters=[{
                "motor_id":       m["motor_id"],
                "spin_up_time_s": m.get("spin_up_time_s", 0.0),
                "move_time_s":    m.get("move_time_s",    1.0),
            }],
            output="screen",
        ))

    # ── Peel sequence orchestrator ──
    nodes.append(Node(
        package="motor_controller",
        executable="peel_sequence_node",
        name="peel_sequence",
        output="screen",
    ))

    return LaunchDescription(nodes)
