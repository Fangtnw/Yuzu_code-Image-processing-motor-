from setuptools import find_packages, setup
import os
from glob import glob

package_name = "motor_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/motor_controller"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/data", glob("data/*.html")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fang",
    maintainer_email="todo@example.com",
    description="ROS2 node for controlling the stepper motor via serial commands.",
    license="MIT",
    entry_points={
        "console_scripts": [
            f"motor_controller_node = {package_name}.motor_controller_node:main",
        ],
    },
)
