"""
vision_bringup.launch.py
────────────────────────────────────────────────────────────────────────────
Launches the complete vision-guided pick and place stack:

  Terminal 1 (already running):
    ros2 launch pca9685_arm_robot robot_bringup.launch.py urdf_file:=...

  Terminal 2 (this launch file):
    ros2 launch vision_pick_place vision_bringup.launch.py

  Terminal 3 (run pick and place):
    ros2 run vision_pick_place vision_pick_place

Nodes started by this launch:
  1. realsense2_camera     — D435 RGB + depth driver
  2. camera_tf_broadcaster — gripper → camera_color_optical_frame TF
  3. green_ball_detector   — HSV detection → /detected_ball_pose
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch", "rs_launch.py"
            )
        ),
        launch_arguments={
            "enable_color":       "true",
            "enable_depth":       "true",
            "align_depth.enable": "true",
            "camera_namespace":   "camera",
            "camera_name":        "camera",
        }.items(),
    )

    camera_tf = Node(
        package="vision_pick_place",
        executable="camera_tf_broadcaster",
        name="camera_tf_broadcaster",
        output="screen",
    )

    detector = Node(
        package="vision_pick_place",
        executable="green_ball_detector",
        name="green_ball_detector",
        parameters=[{"debug": True}],
        output="screen",
    )

    return LaunchDescription([
        realsense_launch,
        camera_tf,
        detector,
    ])