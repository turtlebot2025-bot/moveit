#!/usr/bin/env python3
"""
pick_and_place.py
────────────────────────────────────────────────────────────────────────────
Simple pick and place controller for arm_robot_pkg (5-DOF servo arm).

This node sends JointTrajectory goals directly to arm_controller and
gripper_controller without requiring MoveIt 2. All positions are defined
as joint angles in radians, matched to your calibrated servo ranges.

Usage:
  # In a new terminal (with workspace sourced):
  python3 pick_and_place.py

  # Or as a ROS 2 node:
  ros2 run pca9685_arm_robot pick_and_place

Coordinate convention (matches your URDF):
  base_joint     : + rotates right,   - rotates left     (±1.046 rad)
  shoulder_joint : + raises arm up,   - lowers arm down  (±1.046 rad)
  elbow_joint    : + bends forward,   - bends backward   (±1.046 rad)
  wrist_joint    : + tilts up,        - tilts down       (±1.570 rad)
  gripper_joint  : + closes gripper,  - opens gripper    (±1.570 rad)

IMPORTANT: Verify each pose visually at slow speed before running
the full sequence. Adjust the angles in POSES below to match your
physical setup (object location, drop location, table height, etc).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as RosDuration

import time


# ── Joint name lists (must match controllers.yaml) ────────────────────────────
ARM_JOINTS = [
    "rotating_base_joint",
    "shoulder_link_joint",
    "Elbow_link_joint",
    "wrist_link_joint",
]

GRIPPER_JOINTS = [
    "gripper_joint",
]

# ── Gripper positions ─────────────────────────────────────────────────────────
GRIPPER_OPEN   =  0.0    # rad — fully open   (adjust if needed)
GRIPPER_CLOSED =  1.2    # rad — gripping     (adjust to your object size)

# ── Named arm poses  [base, shoulder, elbow, wrist]  all in radians ───────────
#
# TUNE THESE VALUES to match your physical setup:
#   - HOME:       safe resting position
#   - PRE_PICK:   above the object, arm extended
#   - PICK:       lowered onto the object
#   - LIFT:       raised with object
#   - PRE_PLACE:  above the drop location
#   - PLACE:      lowered to drop location
#   - RETREAT:    raised after placing
#       joint_names: [rotating_base_joint, shoulder_link_joint, Elbow_link_joint, wrist_link_joint]. Order of   joint poses
POSES = {
    "HOME":      [ -0.40, 0.50, -0.6, -0.4],
    "PRE_PICK":  [ -0.70, 1.0, -0.7, -0.60 ],#Position arm above the object. Elbow is is down compared to Lift pose. Once negative value for elbow get increased it get lowered.
    "PICK":      [ -0.70, 1.0, -1.1, -0.60   ],#Shoulder max is 1.046, so this is close to fully down
    "LIFT":      [ -0.70, 1.0, -0.5, -0.60   ],#Lift with object, adjust shoulder and elbow to keep it level
    "PRE_PLACE": [0.90, 1.0, -0.5, -0.60],#Rotate base to place location, keep arm raised
    "PLACE":     [0.90, 1.0, -0.98, 0.4],
    "RETREAT":   [0.90, 1.0, -0.3, -0.4],
}

# ── Motion timing ─────────────────────────────────────────────────────────────
MOVE_DURATION   = 2.0   # seconds per arm movement
GRIPPER_DURATION = 1.0  # seconds per gripper open/close
SETTLE_TIME      = 0.5  # seconds to wait after each move completes


# ── Helper to build a trajectory point ───────────────────────────────────────

def make_point(positions: list, duration_sec: float) -> JointTrajectoryPoint:
    pt = JointTrajectoryPoint()
    pt.positions  = [float(p) for p in positions]
    pt.velocities = [0.0] * len(positions)
    secs     = int(duration_sec)
    nanosecs = int((duration_sec - secs) * 1e9)
    pt.time_from_start = RosDuration(sec=secs, nanosec=nanosecs)
    return pt


# ── Pick and place node ───────────────────────────────────────────────────────

class PickAndPlace(Node):

    def __init__(self):
        super().__init__("pick_and_place")

        # Action clients
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
        )

        self.get_logger().info("Waiting for arm_controller action server...")
        self._arm_client.wait_for_server()
        self.get_logger().info("Waiting for gripper_controller action server...")
        self._gripper_client.wait_for_server()
        self.get_logger().info("Both action servers ready — starting pick and place")

    # ── move helpers ──────────────────────────────────────────────────────────

    def move_arm(self, pose_name: str, duration: float = MOVE_DURATION):
        """Move arm to a named pose."""
        positions = POSES[pose_name]
        self.get_logger().info(
            f"Moving arm → {pose_name}  "
            f"[{', '.join(f'{p:.2f}' for p in positions)}] rad"
        )
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        traj.points = [make_point(positions, duration)]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        time.sleep(SETTLE_TIME)

    def move_gripper(self, position: float, duration: float = GRIPPER_DURATION):
        """Open or close the gripper."""
        label = "CLOSING" if position > 0.1 else "OPENING"
        self.get_logger().info(
            f"Gripper {label} → {position:.2f} rad"
        )
        traj = JointTrajectory()
        traj.joint_names = GRIPPER_JOINTS
        traj.points = [make_point([position], duration)]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        time.sleep(SETTLE_TIME)

    # ── pick and place sequence ───────────────────────────────────────────────

    def run(self):
        self.get_logger().info("=" * 50)
        self.get_logger().info("  Starting pick and place sequence")
        self.get_logger().info("=" * 50)

        # 1. Go to home position
        self.get_logger().info("Step 1 — Home position")
        self.move_arm("HOME")
        self.move_gripper(GRIPPER_OPEN)

        # 2. Move above the pick location
        self.get_logger().info("Step 2 — Move to pre-pick position")
        self.move_arm("PRE_PICK")

        # 3. Lower onto the object
        self.get_logger().info("Step 3 — Lower to pick position")
        self.move_arm("PICK")
        time.sleep(0.3)

        # 4. Close gripper to grasp object
        self.get_logger().info("Step 4 — Close gripper (grasp)")
        self.move_gripper(GRIPPER_CLOSED)
        time.sleep(0.5)   # let gripper settle on object

        # 5. Lift object
        self.get_logger().info("Step 5 — Lift object")
        self.move_arm("LIFT")

        # 6. Move above the place location
        self.get_logger().info("Step 6 — Move to pre-place position")
        self.move_arm("PRE_PLACE")

        # 7. Lower to place location
        self.get_logger().info("Step 7 — Lower to place position")
        self.move_arm("PLACE")
        time.sleep(0.3)

        # 8. Open gripper to release object
        self.get_logger().info("Step 8 — Open gripper (release)")
        self.move_gripper(GRIPPER_OPEN)
        time.sleep(0.3)

        # 9. Retreat from place location
        self.get_logger().info("Step 9 — Retreat")
        self.move_arm("RETREAT")

        # 10. Return home
        self.get_logger().info("Step 10 — Return home")
        self.move_arm("HOME")

        self.get_logger().info("=" * 50)
        self.get_logger().info("  Pick and place sequence complete!")
        self.get_logger().info("=" * 50)


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted — returning to home")
        node.move_arm("HOME")
        node.move_gripper(GRIPPER_OPEN)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()