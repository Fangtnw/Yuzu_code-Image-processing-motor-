"""Launch guarded 0-to-90-degree CW indexing for AZD3A Axis 3."""

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
    package_share = Path(get_package_share_directory("motor_controller"))
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", str(package_share / "urdf" / "azd3a_axis3_index.urdf.xacro")]),
            value_type=str,
        )
    }
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, str(package_share / "config" / "azd3a_axis3_index_controllers.yaml")],
        output="screen",
    )
    nodes = [
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "-c", "/controller_manager"], output="screen"),
        Node(package="controller_manager", executable="spawner", arguments=["axis3_raw_position_controller", "-c", "/controller_manager"], output="screen"),
        Node(
            package="motor_controller",
            executable="azd3a_axis3_index_guard",
            parameters=[{
                "max_rpm": ParameterValue(LaunchConfiguration("max_rpm"), value_type=float),
                "max_acceleration_rpm_s": ParameterValue(LaunchConfiguration("max_acceleration_rpm_s"), value_type=float),
            }],
            output="screen",
        ),
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[robot_description], output="screen"),
    ]
    shutdown = RegisterEventHandler(OnProcessExit(target_action=control_node, on_exit=[EmitEvent(event=Shutdown(reason="ros2_control_node exited"))]))
    return LaunchDescription([
        DeclareLaunchArgument("max_rpm", default_value="5.0"),
        DeclareLaunchArgument("max_acceleration_rpm_s", default_value="5.0"),
        control_node,
        *nodes,
        shutdown,
    ])
