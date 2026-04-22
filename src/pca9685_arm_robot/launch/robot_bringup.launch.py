import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_directory("pca9685_arm_robot")

    # ── Launch arguments ──────────────────────────────────────────────────────
    urdf_arg = DeclareLaunchArgument(
        "urdf_file",
        description="Absolute path to arm_robot_pkg URDF file",
    )
    mock_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="Use mock hardware (no I2C). For desktop testing only.",
    )

    urdf_file         = LaunchConfiguration("urdf_file")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # ── Robot description — wrapped as string to prevent YAML parsing ─────────
    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", urdf_file]),
        value_type=str
    )
    
    robot_description = {"robot_description": robot_description_content}

    controllers_yaml = os.path.join(pkg_share, "config", "controllers.yaml")

    # ── Nodes ─────────────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )


    i2c_node = Node(
    package="pca9685_arm_robot",
    executable="pca9685_i2c_node",
    parameters=[{
        "i2c_bus": 7,
        "i2c_address": 0x40,
        "pwm_frequency": 50.0,
    }],
    output="screen",
)




    arm_gripper_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=jsb_spawner,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=[
                                "arm_controller",
                                "gripper_controller",
                                "--controller-manager", "/controller_manager",
                            ],
                            output="screen",
                        )
                    ],
                )
            ],
        )
    )

    return LaunchDescription([
        urdf_arg,
        mock_arg,
        robot_state_publisher,
        controller_manager,
        jsb_spawner,
        arm_gripper_spawner,
        i2c_node
    ])


# NOTE: append this Node to the return LaunchDescription list manually
# pca9685_i2c_node starts the real hardware driver
