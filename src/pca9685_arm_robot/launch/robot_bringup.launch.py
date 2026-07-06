import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import UnlessCondition
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
        default_value=os.path.join(
            pkg_share, "urdf", "arm_robot_ros2_control_block.urdf"
        ),
        description="Absolute path to the robot URDF file.",
    )
    mock_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description=(
            "Set to 'true' to run without physical I2C hardware "
            "(desktop / CI testing).  pca9685_i2c_node is suppressed."
        ),
    )

    urdf_file         = LaunchConfiguration("urdf_file")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # ── Robot description ─────────────────────────────────────────────────────
    # Wrapped as ParameterValue(str) to prevent the YAML parser inside
    # controller_manager from trying to interpret the URDF XML as YAML.
    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", urdf_file]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    controllers_yaml = os.path.join(pkg_share, "config", "controllers.yaml")

    # ── Core nodes ────────────────────────────────────────────────────────────

    # Publishes TF for all links, including mimic finger links.
    # robot_state_publisher reads the <mimic> tags from the URDF and
    # propagates right_gear_joint's state to the five mimic joints
    # (right_link, left_link, left_gear, right_finger, left_finger).
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ros2_control node — loads the hardware plugin defined in the URDF
    # ros2_control block (pca9685_arm_robot_hardware/PCA9685ArmRobot).
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    # Spawned first; arm/gripper controllers wait for it to be active.
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # arm_controller — spawned 2 s after joint_state_broadcaster is active.
    # Owns: rotating_base_joint, shoulder_joint, elbow_joint,
    #       wrist_joint, gripper_joint (body rotation).
    arm_controller_spawner = RegisterEventHandler(
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
                                "--controller-manager", "/controller_manager",
                            ],
                            output="screen",
                        )
                    ],
                )
            ],
        )
    )

    # gripper_controller — spawned 2.5 s after joint_state_broadcaster.
    # Owns: right_gear_joint (ch 5, finger open/close actuator).
    # The five mimic joints follow right_gear_joint automatically — they are
    # NOT listed in gripper_controller and have no servo of their own.
    gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=jsb_spawner,
            on_start=[
                TimerAction(
                    period=2.5,
                    actions=[
                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=[
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

    # ── Hardware I2C bridge node ───────────────────────────────────────────────
    # pca9685_i2c_node subscribes to /joint_states (published by
    # joint_state_broadcaster) and writes actuated joint positions to the
    # PCA9685 over I2C.  Suppressed when use_mock_hardware:=true.
    #
    # Actuated channel map (must match URDF ros2_control block):
    #   shoulder_joint      → ch 0
    #   elbow_joint         → ch 1
    #   rotating_base_joint → ch 2
    #   wrist_joint         → ch 3
    #   gripper_joint       → ch 4  (body rotation)
    #   right_gear_joint    → ch 5  (finger open/close — PRIMARY actuator)
    #
    # Mimic joints (right_link_joint, left_link_joint, left_gear_joint,
    # right_finger_joint, left_finger_joint) are skipped by this node — their
    # state is computed from right_gear_joint and they have no servo.
    pca9685_i2c_node = Node(
        package="pca9685_arm_robot",
        executable="pca9685_i2c_node",
        name="pca9685_i2c_node",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        parameters=[{
            "i2c_bus":       7,       # Jetson AGX Orin I2C bus for PCA9685
            "i2c_address":   0x40,
            "pwm_frequency": 50.0,
        }],
    )

    return LaunchDescription([
        urdf_arg,
        mock_arg,
        robot_state_publisher,
        controller_manager,
        jsb_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        pca9685_i2c_node,          # real hardware only; skipped if use_mock_hardware:=true
    ])