import os
import glob
from setuptools import find_packages, setup

package_name = "pca9685_arm_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Required: ament resource index marker
        ("share/ament_index/resource_index/packages",
             ["resource/" + package_name]),

        # Required: package manifest
        ("share/" + package_name, ["package.xml"]),

        # pluginlib descriptor
        ("share/" + package_name, ["pca9685_arm_robot_plugin.xml"]),

        # URDF files
        ("share/" + package_name + "/urdf",
             glob.glob("urdf/*.urdf") + glob.glob("urdf/*.xacro")),

        # Mesh files (STL) — required for MoveIt Setup Assistant and RViz
        ("share/" + package_name + "/meshes",
             glob.glob("meshes/*.STL") +
             glob.glob("meshes/*.stl") +
             glob.glob("meshes/*.dae") +
             glob.glob("meshes/*.obj")),

        # Controller and MoveIt config
        ("share/" + package_name + "/config",
             glob.glob("config/*.yaml") + glob.glob("config/*.rviz")),

        # Launch files
        ("share/" + package_name + "/launch",
             glob.glob("launch/*.launch.py") + glob.glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tashen",
    maintainer_email="tashenlrana@gmail.com",
    description=(
        "ros2_control SystemInterface plugin for arm_robot_pkg — "
        "5-DOF servo arm driven by PCA9685 over I2C on Jetson AGX Orin."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [ "pca9685_i2c_node = pca9685_arm_robot.pca9685_i2c_node:main",
                            "pick_and_place    = pca9685_arm_robot.pick_and_place:main",
                            ],
    },
)
