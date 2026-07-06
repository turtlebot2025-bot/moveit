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
        ("share/" + package_name,
             ["package.xml"]),

        # URDF files — picks up arm_robot_ros2_control_block.urdf + any xacro files
        ("share/" + package_name + "/urdf",
             glob.glob("urdf/*.urdf") + glob.glob("urdf/*.xacro")),

        # Mesh files — all formats used by RViz and MoveIt Setup Assistant
        ("share/" + package_name + "/meshes",
             glob.glob("meshes/*.STL") +
             glob.glob("meshes/*.stl") +
             glob.glob("meshes/*.dae") +
             glob.glob("meshes/*.obj")),

        # Config files — controllers.yaml, moveit_controllers.yaml, *.rviz
        ("share/" + package_name + "/config",
             glob.glob("config/*.yaml") + glob.glob("config/*.rviz")),

        # Launch files — use *.py only (avoids duplicating *.launch.py files
        # which are already matched by *.py, preventing double-install warnings)
        ("share/" + package_name + "/launch",
             glob.glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tashen",
    maintainer_email="tashenlrana@gmail.com",
    description=(
        "ROS 2 package for complete_assembly — 5-DOF servo arm with "
        "parallel gripper (right_gear_joint + 5 mimic joints), driven "
        "by a PCA9685 over I2C (bus 7) on the Jetson AGX Orin. "
        "Includes hardware bridge (pca9685_i2c_node), pick_and_place, "
        "servo calibration, and RViz display launch."
    ),
    license="Apache-2.0",
    # Note: tests_require is removed — it was deprecated in modern setuptools
    # and caused "UserWarning: Unknown distribution option: 'tests_require'".
    # pytest is declared as <test_depend> in package.xml instead.
    entry_points={
        "console_scripts": [
            # Hardware I2C bridge — subscribes to /joint_states, drives PCA9685
            "pca9685_i2c_node  = pca9685_arm_robot.pca9685_i2c_node:main",

            # Hardcoded pick and place sequence
            "pick_and_place     = pca9685_arm_robot.pick_and_place:main",

            # Interactive servo calibration tool (no ROS needed)
            # Usage: ros2 run pca9685_arm_robot calibrate_servos --channel 2
            "calibrate_servos   = pca9685_arm_robot.calibrate_servos:main",
        ],
    },
)