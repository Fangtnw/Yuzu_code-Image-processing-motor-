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
        ],
    },
)
