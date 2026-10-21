"""Launch the integrated six-motor Yuzu peeler control system and GUI."""

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
                ["xacro ", str(package_share / "urdf" / "azd3a_motor1_motor2.urdf.xacro")]
            ),
            value_type=str,
        )
    }
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            str(package_share / "config" / "azd3a_motor1_motor2_controllers.yaml"),
        ],
        output="screen",
    )
    nodes = [
        control_node,
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["motor1_raw_position_controller", "-c", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["motor2_raw_velocity_controller", "-c", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["motor4_raw_position_controller", "-c", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["motor3_raw_position_controller", "-c", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["motor6_raw_velocity_controller", "-c", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["motor5_raw_position_controller", "-c", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="motor_controller",
            executable="azd3a_motor1_position_guard",
            parameters=[{
                "feedback_joint_name": "motor1_motor2",
                "min_position_m": 0.0,
                "max_position_m": 0.400,
                "max_velocity_m_s": 0.480,
                "max_acceleration_m_s2": 0.480,
            }],
            output="screen",
        ),
        Node(
            package="motor_controller",
            executable="azd3a_motor2_velocity_guard",
            parameters=[{
                "max_rpm": 332.8,
                "max_acceleration_rpm_s": 665.6,
                "command_timeout_s": 0.5,
            }],
            output="screen",
        ),
        Node(
            package="motor_controller",
            executable="azd3a_motor4_position_guard",
            output="screen",
        ),
        Node(
            package="motor_controller",
            executable="azd3a_motor3_position_guard",
            output="screen",
        ),
        Node(
            package="motor_controller",
            executable="azd3a_motor6_conveyor_guard",
            parameters=[{
                "max_rpm": 48.0,
                "max_acceleration_rpm_s": 96.0,
                "command_timeout_s": 0.5,
            }],
            output="screen",
        ),
        Node(
            package="motor_controller",
            executable="azd3a_motor5_position_guard",
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(package="motor_controller", executable="yuzu_peeler_gui", output="screen"),
        RegisterEventHandler(
            OnProcessExit(
                target_action=control_node,
                on_exit=[EmitEvent(event=Shutdown(reason="ros2_control_node exited"))],
            )
        ),
    ]
    return LaunchDescription(nodes)
