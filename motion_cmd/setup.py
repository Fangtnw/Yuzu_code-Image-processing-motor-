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
    maintainer="fang",
    maintainer_email="todo@example.com",
    description="ROS2 utilities for one-axis AZD3A-KED EtherCAT bring-up.",
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
        ],
    },
)
