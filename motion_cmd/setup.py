from setuptools import find_packages, setup
from glob import glob

package_name = "motor_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/motor_controller"]),
        (f"share/{package_name}", ["package.xml", *glob("*.md")]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/urdf", glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yuzu Peeler Team",
    maintainer_email="fang@users.noreply.github.com",
    description="Guarded ROS 2 control, EtherCAT mappings, and operator GUI for the six-motor Yuzu peeler.",
    license="MIT",
    entry_points={
        "console_scripts": [
            f"azd3a_basic_test           = {package_name}.azd3a_basic_test_node:main",
            f"azd3a_ethercat_check       = {package_name}.azd3a_ethercat_check:main",
            f"azd3a_axis1_video_demo     = {package_name}.azd3a_axis1_video_demo:main",
            f"azd3a_axis1_command_guard  = {package_name}.azd3a_axis1_command_guard:main",
            f"azd3a_axis2_velocity_guard = {package_name}.azd3a_axis2_velocity_guard:main",
            f"azd3a_axis3_velocity_guard = {package_name}.azd3a_axis3_velocity_guard:main",
            f"azd3a_axis3_index_guard    = {package_name}.azd3a_axis3_index_guard:main",
            f"azd3a_motor4_commissioning_guard = {package_name}.azd3a_motor4_commissioning_guard:main",
            f"azd3a_motor3_commissioning_guard = {package_name}.azd3a_motor3_commissioning_guard:main",
            f"azd3a_motor3_position_guard = {package_name}.azd3a_motor3_position_guard:main",
            f"azd3a_motor5_commissioning_guard = {package_name}.azd3a_motor5_commissioning_guard:main",
            f"azd3a_motor5_position_guard = {package_name}.azd3a_motor5_position_guard:main",
            f"azd3a_motor4_position_guard = {package_name}.azd3a_motor4_position_guard:main",
            f"azd3a_motor6_conveyor_guard = {package_name}.azd3a_motor6_conveyor_guard:main",
            f"yuzu_operator_gui          = {package_name}.yuzu_operator_gui:main",
        ],
    },
)
