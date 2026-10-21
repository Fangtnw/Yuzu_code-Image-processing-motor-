"""Launch guarded commissioning control for AZD3A Axis 3."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    max_rpm = LaunchConfiguration("max_rpm")
    max_acceleration_rpm_s = LaunchConfiguration("max_acceleration_rpm_s")
    command_timeout_s = LaunchConfiguration("command_timeout_s")
    package_share = Path(get_package_share_directory("motor_controller"))
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                ["xacro ", str(package_share / "urdf" / "azd3a_axis3_tiny_spin.urdf.xacro")]
            ),
            value_type=str,
        )
    }
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            str(package_share / "config" / "azd3a_axis3_tiny_spin_controllers.yaml"),
        ],
        output="screen",
    )
    state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["axis3_raw_velocity_controller", "-c", "/controller_manager"],
        output="screen",
    )
    command_guard = Node(
        package="motor_controller",
        executable="azd3a_axis3_velocity_guard",
        parameters=[
            {
                "max_rpm": ParameterValue(max_rpm, value_type=float),
                "max_acceleration_rpm_s": ParameterValue(
                    max_acceleration_rpm_s, value_type=float
                ),
                "command_timeout_s": ParameterValue(command_timeout_s, value_type=float),
            }
        ],
        output="screen",
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )
    shutdown_on_control_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[EmitEvent(event=Shutdown(reason="ros2_control_node exited"))],
        )
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "max_rpm",
                default_value="5.0",
                description="Guarded absolute Axis 3 speed ceiling in rpm",
            ),
            DeclareLaunchArgument(
                "max_acceleration_rpm_s",
                default_value="2.0",
                description="Axis 3 command ramp in rpm/s",
            ),
            DeclareLaunchArgument(
                "command_timeout_s",
                default_value="0.5",
                description="Time without a public RPM command before ramping to zero",
            ),
            control_node,
            state_broadcaster,
            velocity_controller,
            command_guard,
            state_publisher,
            shutdown_on_control_exit,
        ]
    )
