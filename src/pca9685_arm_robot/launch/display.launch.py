"""
display_launch.py
────────────────────────────────────────────────────────────────────────────
RViz display launch for complete_assembly robot arm.

Starts:
  1. robot_state_publisher   — publishes TF for all 13 links
                               (mimic joints propagated via URDF <mimic> tags)
  2. joint_state_publisher_gui — interactive sliders for manual joint testing
                               (use_mimic_tags=True: moving right_gear_joint
                                slider animates all 5 finger mimic joints)
  3. rviz2                   — 3D visualisation

Usage:
  ros2 launch pca9685_arm_robot display_launch.py

Note: This launch is for DISPLAY/TESTING only. It does not start
ros2_control or the PCA9685 hardware driver. For the full robot
bringup, use robot_bringup.launch.py instead.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("pca9685_arm_robot")

    # ── URDF file ─────────────────────────────────────────────────────────────
    # Uses complete_assembly.urdf which contains the full kinematic chain
    # including parallel gripper with mimic joints.
    urdf_file = os.path.join(pkg_share, "urdf", "arm_robot_ros2_control_block.urdf")

    if not os.path.isfile(urdf_file):
        sys.exit(
            f"\n[display_launch] ERROR: URDF not found at:\n  {urdf_file}\n"
            f"Make sure arm_robot_ros2_control_block.urdf is in the urdf/ folder and "
            f"the package has been built with:\n"
            f"  colcon build --packages-select pca9685_arm_robot --symlink-install\n"
        )

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    # ── RViz config ───────────────────────────────────────────────────────────
    # Use package-provided config if it exists; otherwise launch RViz without
    # a config (user can configure manually and save).
    rviz_config = os.path.join(pkg_share, "config", "complete_assembly.rviz")
    rviz_args   = ["-d", rviz_config] if os.path.isfile(rviz_config) else []

    if not rviz_args:
        print(
            "[display_launch] WARNING: No RViz config found at:\n"
            f"  {rviz_config}\n"
            "  RViz will start with default settings.\n"
            "  To save a config: File → Save Config As → "
            "config/complete_assembly.rviz"
        )

    # ── Nodes ─────────────────────────────────────────────────────────────────

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # joint_state_publisher_gui provides interactive sliders.
    # use_mimic_tags=True honours the URDF <mimic> declarations:
    #   moving right_gear_joint slider → right_link, left_link,
    #   left_gear, right_finger, left_finger all move together.
    # source_list=[] makes the GUI the sole joint state source
    # (no passthrough of external /joint_states in display mode).
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[
            {"robot_description": robot_description},
            {"use_mimic_tags":    True},
            {"source_list":       []},
        ],
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=rviz_args,
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])