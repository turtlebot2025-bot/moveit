import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('complete_assembly')

    urdf_file = os.path.join(pkg_share, 'urdf', 'complete_assembly.urdf')
    rviz_config_file = os.path.join(pkg_share, 'config', 'complete_assembly.rviz')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[
                {'robot_description': robot_description},
                # Honour <mimic> tags for the gripper linkage
                {'use_mimic_tags': True},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
        )
    ])
