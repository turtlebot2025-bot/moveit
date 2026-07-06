#!/usr/bin/env python3
"""
pick_and_place.py  (fixed — uses freeze/unfreeze to hard-stop base servo)
────────────────────────────────────────────────────────────────────────────
Works together with the fixed pca9685_i2c_node.py.

After RETREAT:
  1. Publishes "rotating_base_joint" to /pca9685_arm_robot/freeze_joints
     → i2c_node immediately hard-writes current pulse and ignores all
       further /joint_states updates for that joint.
  2. Shoulder is moved clearly (+1.20 rad) during RETREAT for visible lift.
  3. Base-only move to home while shoulder stays raised.
  4. Unfreeze base before lowering arm to home.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as RosDuration
from control_msgs.msg import JointTolerance


# ── Controller joint lists ────────────────────────────────────────────────────
ARM_JOINTS    = [
    "rotating_base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_joint",
    "gripper_joint",
]
FINGER_JOINTS = ["right_gear_joint"]

# ── Gripper ───────────────────────────────────────────────────────────────────
FINGER_OPEN   = -0.5
FINGER_CLOSED =  1.0

# ── Poses  [base, shoulder, elbow, wrist, gripper_body] ──────────────────────
POSES = {
    # True neutral
    "HOME":         [  0.00,  0.30, -0.30, -0.20,  0.00 ],

    # Pick side — base stays at +0.70 for PICK and LIFT
    "PRE_PICK":     [  0.70,  0.42, -0.70, -0.60,  0.00 ],
    "PICK":         [  0.70,  0.42, -1.10, -0.60,  0.08 ],
    "LIFT":         [  0.70,  1.00, -0.50, -0.40,  0.08 ],

    # Place side — base stays at -0.90 for PRE_PLACE, PLACE, RETREAT
    "PRE_PLACE":    [ -0.90,  -0.80, -0.50, -0.40,  0.00 ],
    "PLACE":        [ -0.90,  -0.90, -0.98,  0.40,  0.00 ],

    # RETREAT: base FROZEN at -0.90, shoulder raises to +1.20 (clear lift)
    "RETREAT":      [ -0.90,  1.20, -0.40, -0.30,  0.00 ],

    # Base-only move to 0.0 while shoulder stays raised at +1.20
    "BASE_TO_HOME": [  0.00,  1.20, -0.40, -0.30,  0.00 ],

    # Lower arm to final home (base already at 0)
    "ARM_TO_HOME":  [  0.00,  0.30, -0.30, -0.20,  0.00 ],
}

# ── Timing ────────────────────────────────────────────────────────────────────
T_MOVE    = 3.5
T_FAST    = 0.3   # re-lock duration
T_BASE    = 5.0   # slow base-only rotation
T_FINGER  = 1.0
T_SETTLE  = 0.5


# ── Trajectory helper ─────────────────────────────────────────────────────────

def make_point(positions, duration_sec):
    pt = JointTrajectoryPoint()
    pt.positions  = [float(p) for p in positions]
    pt.velocities = [0.0] * len(positions)
    s  = int(duration_sec)
    ns = int((duration_sec - s) * 1e9)
    pt.time_from_start = RosDuration(sec=s, nanosec=ns)
    return pt


def send_goal(node, client, joint_names, positions, duration,
              label, tol=0.08):
    traj = JointTrajectory()
    traj.joint_names = joint_names
    traj.points = [make_point(positions, duration)]

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj

    for name in joint_names:
        jt = JointTolerance()
        jt.name     = name
        jt.position = tol
        jt.velocity = 0.1
        goal.goal_tolerance.append(jt)
    goal.goal_time_tolerance = RosDuration(sec=2, nanosec=0)

    node.get_logger().info(
        f"  → {label}  [{', '.join(f'{p:.2f}' for p in positions)}]"
    )
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    gh = fut.result()
    if not gh.accepted:
        node.get_logger().error(f"Goal REJECTED: {label}")
        return False

    res_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut)
    time.sleep(T_SETTLE)
    return True


# ── Main node ─────────────────────────────────────────────────────────────────

class PickAndPlace(Node):

    def __init__(self):
        super().__init__("pick_and_place")

        self._arm = ActionClient(
            self, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )
        self._grip = ActionClient(
            self, FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory"
        )

        # Publishers to freeze/unfreeze joints in pca9685_i2c_node
        self._freeze_pub = self.create_publisher(
            String, "/pca9685_arm_robot/freeze_joints", 10
        )
        self._unfreeze_pub = self.create_publisher(
            String, "/pca9685_arm_robot/unfreeze_joints", 10
        )

        self.get_logger().info("Waiting for action servers...")
        self._arm.wait_for_server()
        self._grip.wait_for_server()
        self.get_logger().info("Ready")

    # ── Primitives ────────────────────────────────────────────────────────────

    def arm(self, pose_name, duration=T_MOVE, tol=0.08):
        self.get_logger().info(f"ARM → {pose_name}")
        send_goal(self, self._arm, ARM_JOINTS,
                  POSES[pose_name], duration, pose_name, tol)

    def fingers(self, pos):
        label = "CLOSE" if pos > 0 else "OPEN"
        self.get_logger().info(f"FINGERS {label} → {pos:.2f} rad")
        send_goal(self, self._grip, FINGER_JOINTS,
                  [pos], T_FINGER, label)

    def freeze(self, *joint_names):
        """
        Hard-stop specified joints: i2c_node writes current pulse once
        then ignores /joint_states for those joints until unfreeze.
        """
        msg = String()
        msg.data = ",".join(joint_names)
        self._freeze_pub.publish(msg)
        self.get_logger().info(f"FREEZE → {msg.data}")
        time.sleep(0.1)   # give i2c_node time to process

    def unfreeze(self, *joint_names):
        """Resume /joint_states updates for specified joints."""
        msg = String()
        msg.data = ",".join(joint_names)
        self._unfreeze_pub.publish(msg)
        self.get_logger().info(f"UNFREEZE → {msg.data}")
        time.sleep(0.1)

    # ── Sequence ──────────────────────────────────────────────────────────────

    def run(self):
        log = self.get_logger().info
        log("══════════════════════════════════════════")
        log("  Pick-and-place START")
        log("══════════════════════════════════════════")

        # 1. Home + open gripper
        log("Step 1 — Home")
        self.unfreeze("rotating_base_joint")   # ensure not frozen from prev run
        self.arm("HOME")
        self.fingers(FINGER_OPEN)
        time.sleep(0.5)

        # 2. Pre-pick approach
        log("Step 2 — Pre-pick")
        self.arm("PRE_PICK")

        # 3. Descend to object
        log("Step 3 — Pick (lower)")
        self.arm("PICK")
        time.sleep(0.4)

        # 4. Grasp
        log("Step 4 — Grasp")
        self.fingers(FINGER_CLOSED)
        time.sleep(0.6)

        # 5. Lift
        log("Step 5 — Lift")
        self.arm("LIFT")

        # 6. Swing to place side
        log("Step 6 — Pre-place")
        self.arm("PRE_PLACE")

        # 7. Lower to drop
        log("Step 7 — Place (lower)")
        self.arm("PLACE")
        time.sleep(0.4)

        # 8. Release
        log("Step 8 — Release")
        self.fingers(FINGER_OPEN)
        time.sleep(0.4)

        # ── RETREAT with base freeze ──────────────────────────────────────────
        # 9. Start RETREAT trajectory — shoulder raises to +1.20
        log("Step 9 — Retreat (shoulder raises to +1.20 rad)")
        self.arm("RETREAT", duration=T_MOVE)

        # 10. FREEZE base immediately — hard-stops at -0.90
        log("Step 10 — Freeze base at -0.90 rad")
        self.freeze("rotating_base_joint")

        # Wait for shoulder and elbow to fully settle at RETREAT position
        time.sleep(1.0)

        # 11. Base-only rotation to 0.0 (unfreeze base, shoulder stays high)
        log("Step 11 — Unfreeze base, rotate to home (shoulder stays raised)")
        self.unfreeze("rotating_base_joint")
        time.sleep(0.1)   # brief gap so unfreeze processes before new goal
        self.arm("BASE_TO_HOME", duration=T_BASE, tol=0.10)

        # 12. Freeze base at 0.0
        log("Step 12 — Freeze base at 0.0 rad")
        self.freeze("rotating_base_joint")
        time.sleep(0.5)

        # 13. Lower arm to home
        log("Step 13 — Lower arm to home")
        self.unfreeze("rotating_base_joint")
        self.arm("ARM_TO_HOME", duration=T_MOVE)

        log("══════════════════════════════════════════")
        log("  Sequence COMPLETE")
        log("══════════════════════════════════════════")


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted — freezing all joints then homing")
        node.freeze("rotating_base_joint")
        time.sleep(0.3)
        node.unfreeze("rotating_base_joint")
        node.arm("HOME", duration=5.0)
        node.fingers(FINGER_OPEN)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()