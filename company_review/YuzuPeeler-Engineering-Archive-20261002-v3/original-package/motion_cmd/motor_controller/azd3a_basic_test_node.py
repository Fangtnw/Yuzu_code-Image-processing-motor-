"""CLI publisher for one-axis AZD3A-KED ROS2 bring-up tests."""

import argparse
import time

from .azd3a_basic_commands import (
    AxisLimits,
    BasicCommand,
    build_angle_command,
    build_move_mm_command,
    build_spin_rpm_command,
)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Publish one safe basic motor command for AZD3A-KED ROS2 tests."
    )
    parser.add_argument(
        "--topic-prefix",
        default="",
        help="Optional ROS topic prefix, for example /axis1 or /azd_test.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the command instead of publishing it.",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("home", help="Publish std_msgs/Empty to motor_home.")
    subparsers.add_parser("stop", help="Publish std_msgs/Empty to motor_stop.")
    subparsers.add_parser(
        "reset-alarm",
        help="Publish std_msgs/Empty to motor_reset_alarm for drivers that support it.",
    )

    move = subparsers.add_parser("move-mm", help="Absolute linear move in mm.")
    move.add_argument("position_mm", type=float)
    move.add_argument("speed_mm_s", type=float)
    move.add_argument("--accel-mm-s2", type=float, default=0.0)
    move.add_argument("--decel-mm-s2", type=float)
    move.add_argument("--min-position-mm", type=float, default=-1000.0)
    move.add_argument("--max-position-mm", type=float, default=1000.0)
    move.add_argument("--max-speed-mm-s", type=float, default=1000.0)

    spin = subparsers.add_parser("spin-rpm", help="Rotary speed command in rpm.")
    spin.add_argument("rpm", type=float)
    spin.add_argument("--accel-rpm-s", type=float, default=0.0)
    spin.add_argument("--decel-rpm-s", type=float)
    spin.add_argument("--min-rpm", type=float, default=-3000.0)
    spin.add_argument("--max-rpm", type=float, default=3000.0)

    angle = subparsers.add_parser("move-angle", help="Absolute rotary angle command.")
    angle.add_argument("angle_deg", type=float)
    angle.add_argument("speed_rpm", type=float)
    angle.add_argument("--accel-rpm-s", type=float, default=0.0)
    angle.add_argument("--decel-rpm-s", type=float)
    angle.add_argument("--min-angle-deg", type=float, default=-360000.0)
    angle.add_argument("--max-angle-deg", type=float, default=360000.0)
    angle.add_argument("--max-rpm", type=float, default=3000.0)

    return parser


def _topic(prefix: str, name: str) -> str:
    clean_prefix = prefix.strip("/")
    if not clean_prefix:
        return name
    return f"{clean_prefix}/{name}"


def _command_from_args(args: argparse.Namespace) -> BasicCommand:
    if args.command == "move-mm":
        limits = AxisLimits(
            min_position_mm=args.min_position_mm,
            max_position_mm=args.max_position_mm,
            max_speed_mm_s=args.max_speed_mm_s,
        )
        return build_move_mm_command(
            position_mm=args.position_mm,
            speed_mm_s=args.speed_mm_s,
            accel_mm_s2=args.accel_mm_s2,
            decel_mm_s2=args.decel_mm_s2,
            limits=limits,
        )

    if args.command == "spin-rpm":
        limits = AxisLimits(min_rpm=args.min_rpm, max_rpm=args.max_rpm)
        return build_spin_rpm_command(
            rpm=args.rpm,
            accel_rpm_s=args.accel_rpm_s,
            decel_rpm_s=args.decel_rpm_s,
            limits=limits,
        )

    if args.command == "move-angle":
        limits = AxisLimits(
            min_angle_deg=args.min_angle_deg,
            max_angle_deg=args.max_angle_deg,
            max_rpm=args.max_rpm,
        )
        return build_angle_command(
            angle_deg=args.angle_deg,
            speed_rpm=args.speed_rpm,
            accel_rpm_s=args.accel_rpm_s,
            decel_rpm_s=args.decel_rpm_s,
            limits=limits,
        )

    if args.command == "home":
        return BasicCommand("motor_home", "Empty", [])
    if args.command == "stop":
        return BasicCommand("motor_stop", "Empty", [])
    if args.command == "reset-alarm":
        return BasicCommand("motor_reset_alarm", "Empty", [])

    raise ValueError(f"Unsupported command: {args.command}")


def _print_dry_run(args: argparse.Namespace, command: BasicCommand) -> None:
    topic = _topic(args.topic_prefix, command.topic)
    print(f"command: {args.command}")
    print(f"topic: {topic}")
    print(f"type: std_msgs/msg/{command.message_type}")
    print(f"values: {command.values}")


def _publish(command: BasicCommand, topic_name: str) -> None:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Empty, Float32MultiArray

    rclpy.init()
    node = Node("azd3a_basic_test")
    try:
        if command.message_type == "Empty":
            publisher = node.create_publisher(Empty, topic_name, 10)
            message = Empty()
        else:
            publisher = node.create_publisher(Float32MultiArray, topic_name, 10)
            message = Float32MultiArray()
            message.data = command.values

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        publisher.publish(message)
        node.get_logger().info(f"Published {command.message_type} to {topic_name}")
        rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    try:
        command = _command_from_args(args)
    except ValueError as exc:
        parser.error(str(exc))

    if args.dry_run:
        _print_dry_run(args, command)
        return 0

    _publish(command, _topic(args.topic_prefix, command.topic))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
