"""Launch read-only ROS feedback for AZD3A Axis 3."""

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
    xacro_file = package_share / "urdf" / "azd3a_axis3_feedback.urdf.xacro"
    controllers_file = (
        package_share / "config" / "azd3a_axis3_feedback_controllers.yaml"
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
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )
    shutdown_state_publisher = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[EmitEvent(event=Shutdown(reason="ros2_control_node exited"))],
        )
    )

    return LaunchDescription(
        [
            control_node,
            state_broadcaster,
            state_publisher,
            shutdown_state_publisher,
        ]
    )
