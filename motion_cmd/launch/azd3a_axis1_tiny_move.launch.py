"""Launch the AZD3A Axis 1 tiny-motion configuration."""

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
    xacro_file = package_share / "urdf" / "azd3a_axis1_tiny_move.urdf.xacro"
    controllers_file = (
        package_share / "config" / "azd3a_axis1_tiny_move_controllers.yaml"
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", str(xacro_file)]),
            value_type=str,
        )
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, str(controllers_file)],
        output="screen",
    )

    state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["axis1_raw_position_controller", "-c", "/controller_manager"],
        output="screen",
    )

    command_guard = Node(
        package="motor_controller",
        executable="azd3a_axis1_command_guard",
        parameters=[
            {
                # Provisional machine coordinates: the captured lower end is
                # zero and positive ROS motion is upward. Keep the first
                # assembled-machine envelope to half the actuator stroke.
                # Never command the captured mechanical-end coordinate itself.
                "min_position_m": 0.0000996,
                "max_position_m": 0.200,
                "max_velocity_m_s": 0.015,
                "max_acceleration_m_s2": 0.005,
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
            position_controller,
            command_guard,
            state_publisher,
            shutdown_on_control_exit,
        ]
    )
