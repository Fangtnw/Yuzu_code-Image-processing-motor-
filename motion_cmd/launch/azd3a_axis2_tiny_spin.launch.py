"""Launch guarded 25 rpm commissioning control for AZD3A Axis 2."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("motor_controller"))
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                ["xacro ", str(package_share / "urdf" / "azd3a_axis2_tiny_spin.urdf.xacro")]
            ),
            value_type=str,
        )
    }
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            str(package_share / "config" / "azd3a_axis2_tiny_spin_controllers.yaml"),
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
        arguments=["axis2_raw_velocity_controller", "-c", "/controller_manager"],
        output="screen",
    )
    command_guard = Node(
        package="motor_controller",
        executable="azd3a_axis2_velocity_guard",
        parameters=[
            {
                "max_rpm": 25.0,
                "max_acceleration_rpm_s": 10.0,
                "command_timeout_s": 0.5,
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
            control_node,
            state_broadcaster,
            velocity_controller,
            command_guard,
            state_publisher,
            shutdown_on_control_exit,
        ]
    )
