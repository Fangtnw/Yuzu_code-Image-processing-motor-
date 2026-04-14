"""
Launch — EtherNet/IP (LAN) control (ethernet_ip_motor_node).

Usage:
    ros2 launch motor_controller motor_lan.launch.py

Axis map:
  motor_z              — Z-axis linear (DR28T)    ← EtherNet/IP via ethernet_ip_motor_node
  motor_yuzu_rot       — Yuzu rotation (rotary)   ← stub
  motor_peeler_orbit   — Peeler orbit (rotary)    ← stub
  motor_peeler_advance — Peeler advance (linear)  ← stub (second DR28T, LAN)
  motor_conveyor       — Conveyor belt (indexed)  ← stub

Hardware setup:
  1. Set the AZD-KEP IP address using the rotary switches on the driver face:
       IP ADDR ×16 and ×1 dials → IP = 192.168.1.(16×x16 + x1)
       e.g. ×16=0, ×1=2  →  192.168.1.2
  2. Connect a LAN cable: PC ↔ driver (direct or via network switch).
  3. Set PC NIC to the same subnet:
       IP      : 192.168.1.100
       Mask    : 255.255.255.0
       Gateway : (leave blank for direct connection)
  4. Install pycomm3 (once):
       pip install pycomm3
  5. colcon build && ros2 launch motor_controller motor_lan.launch.py

Calibration:
  Set steps_per_mm to match your DR28T configuration:
    steps_per_mm = (encoder_pulses_per_rev) / (screw_lead_mm_per_rev)
  You can also tune at runtime without rebuilding:
    ros2 param set /motor_z/ethernet_ip_motor_node steps_per_mm 500.0
"""

from launch import LaunchDescription
from launch_ros.actions import Node

# ── EtherNet/IP linear motors (AZD-KEP) ───────────────────────────────────────
# Add one entry per DR28T connected via LAN.
ETHERNET_IP_MOTORS = [
    {
        "ns":           "motor_z",
        "motor_id":     "motor_z",
        "ip":           "192.168.1.2",   # !! must match driver rotary switch
        "steps_per_mm": 100.0,           # !! CALIBRATE before first run
    },
    # Second DR28T (peeler advance) — uncomment when connected:
    # {
    #     "ns":           "motor_peeler_advance",
    #     "motor_id":     "motor_peeler_advance",
    #     "ip":           "192.168.1.3",
    #     "steps_per_mm": 100.0,
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

    # ── EtherNet/IP motors ──
    for m in ETHERNET_IP_MOTORS:
        nodes.append(Node(
            package="motor_controller",
            executable="ethernet_ip_motor_node",
            name="ethernet_ip_motor_node",
            namespace=m["ns"],
            parameters=[{
                "ip_address":   m["ip"],
                "motor_id":     m["motor_id"],
                "steps_per_mm": m.get("steps_per_mm", 100.0),
            }],
            output="screen",
        ))

    # ── Stubs for motors not yet connected ──
    lan_namespaces = {m["ns"] for m in ETHERNET_IP_MOTORS}
    for m in ROTARY_STUBS:
        if m["ns"] in lan_namespaces:
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
