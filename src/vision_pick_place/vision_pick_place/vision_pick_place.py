#!/usr/bin/env python3
"""
vision_pick_place.py  (fixed)
────────────────────────────────────────────────────────────────────────────
Vision-guided pick and place with:
  - Averaged ball pose over N readings (eliminates jitter)
  - Robust TF retry loop
  - Sanity checks on detected coordinates
  - Clear error messages for each failure mode
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

import numpy as np

from geometry_msgs.msg import PointStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as RosDuration

import tf2_ros
import tf2_geometry_msgs


# ── Arm geometry (metres) — measure from your physical robot ─────────────────
L1 = 0.120           # shoulder → elbow (metres)
L2 = 0.095           # elbow → wrist (metres)
GRIPPER_LENGTH = 0.060

# ── Workspace limits — reject detections outside these bounds ─────────────────
# In base_link frame (metres). Adjust to match your table layout.
MIN_X, MAX_X =  0.05,  0.35   # forward reach
MIN_Y, MAX_Y = -0.30,  0.30   # lateral
MIN_Z, MAX_Z = -0.10,  0.20   # height (negative = below base origin)

# ── Pick Z offset above ball centre ──────────────────────────────────────────
PICK_APPROACH_OFFSET = 0.08   # metres — approach from this height above ball

# ── Ball size estimate for Z correction ──────────────────────────────────────
BALL_RADIUS = 0.035   # metres — subtract from Z to target ball centre

# ── Pose averaging ────────────────────────────────────────────────────────────
N_READINGS    = 15    # collect this many valid readings before averaging
MAX_WAIT_SEC  = 15.0  # give up after this many seconds

# ── Joint limits ──────────────────────────────────────────────────────────────
BASE_LIMIT     = 1.046
SHOULDER_LIMIT = 1.046
ELBOW_LIMIT    = 1.046
WRIST_LIMIT    = 1.570

# ── Named poses ───────────────────────────────────────────────────────────────
POSE_HOME  = [0.00,  0.00,  0.00,  0.00]
POSE_SCAN  = [0.00,  0.25, -0.40, -0.20]
POSE_PLACE = [-0.60, 0.30, -0.30, -0.20]

GRIPPER_OPEN   =  0.0
GRIPPER_CLOSED =  1.2

#Seconds to spend on each motion — adjust as needed for your robot's speed
T_MOVE    = 2.5
T_PICK    = 1.8
T_GRIPPER = 1.0
T_SETTLE  = 0.4

ARM_JOINTS     = ["rotating_base_joint", "shoulder_link_joint", "Elbow_link_joint", "wrist_link_joint"]
GRIPPER_JOINTS = ["gripper_joint"]
CAMERA_FRAME   = "camera_color_optical_frame"


# ── Helpers ───────────────────────────────────────────────────────────────────

def make_point(positions, duration_sec):
    pt = JointTrajectoryPoint()
    pt.positions  = [float(p) for p in positions]
    pt.velocities = [0.0] * len(positions)
    s = int(duration_sec)
    ns = int((duration_sec - s) * 1e9)
    pt.time_from_start = RosDuration(sec=s, nanosec=ns)
    return pt


def clamp(value, limit):
    return max(-limit, min(limit, value))


def planar_ik(r, z, l1=L1, l2=L2):
    """2-link planar IK. Returns (shoulder, elbow) or raises ValueError."""
    reach = math.sqrt(r**2 + z**2)
    if reach > l1 + l2:
        raise ValueError(
            f"Target out of reach: distance {reach:.3f}m > max {l1+l2:.3f}m"
        )
    if reach < abs(l1 - l2):
        raise ValueError(
            f"Target too close: distance {reach:.3f}m < min {abs(l1-l2):.3f}m"
        )
    D = (r**2 + z**2 - l1**2 - l2**2) / (2 * l1 * l2)
    D = max(-1.0, min(1.0, D))
    elbow    = -math.acos(D)
    k1       = l1 + l2 * math.cos(elbow)
    k2       = l2 * math.sin(elbow)
    shoulder = math.atan2(z, r) - math.atan2(k2, k1)
    return shoulder, elbow


# ── Main node ─────────────────────────────────────────────────────────────────

class VisionPickPlace(Node):

    def __init__(self):
        super().__init__("vision_pick_place")

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory"
        )

        self._raw_readings = []   # list of PointStamped in camera frame

        self._ball_sub = self.create_subscription(
            PointStamped,
            "/detected_ball_pose",
            self._on_ball,
            10,
        )

        self.get_logger().info("Waiting for action servers...")
        self._arm_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info("Action servers ready")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_ball(self, msg: PointStamped):
        self._raw_readings.append(msg)

    # ── Move helpers ──────────────────────────────────────────────────────────

    def _send_arm(self, positions, duration=T_MOVE):
        label = [f"{p:.2f}" for p in positions]
        self.get_logger().info(f"  Arm → [{', '.join(label)}] rad")
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        traj.points = [make_point(positions, duration)]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        rclpy.spin_until_future_complete(self, fut.result().get_result_async())
        time.sleep(T_SETTLE)

    def _send_gripper(self, position, duration=T_GRIPPER):
        label = "CLOSE" if position > 0.1 else "OPEN"
        self.get_logger().info(f"  Gripper {label} → {position:.2f} rad")
        traj = JointTrajectory()
        traj.joint_names = GRIPPER_JOINTS
        traj.points = [make_point([position], duration)]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        rclpy.spin_until_future_complete(self, fut.result().get_result_async())
        time.sleep(T_SETTLE)

    # ── TF transform with retry ───────────────────────────────────────────────

    def _transform_to_base(self, point_stamped, retries=5):
        for attempt in range(retries):
            try:
                return self._tf_buffer.transform(
                    point_stamped,
                    "base_link",
                    timeout=Duration(seconds=1.0),
                )
            except tf2_ros.LookupException:
                self.get_logger().warn(
                    f"TF lookup failed (attempt {attempt+1}/{retries}) — "
                    f"frame '{point_stamped.header.frame_id}' not found in tree.\n"
                    f"Ensure camera_tf_broadcaster is running and gripper→camera_link TF is published."
                )
            except tf2_ros.ConnectivityException as e:
                self.get_logger().warn(
                    f"TF connectivity error (attempt {attempt+1}/{retries}): {e}\n"
                    f"The two TF trees (robot + camera) are not connected.\n"
                    f"Check camera_tf_broadcaster is running with correct PARENT_FRAME='gripper'."
                )
            except tf2_ros.ExtrapolationException as e:
                self.get_logger().warn(
                    f"TF extrapolation (attempt {attempt+1}/{retries}): {e}"
                )
            except Exception as e:
                self.get_logger().warn(
                    f"TF error (attempt {attempt+1}/{retries}): {e}"
                )
            time.sleep(0.3)
        return None

    # ── Averaged ball detection ───────────────────────────────────────────────

    def _get_stable_ball_pose(self):
        """
        Collect N_READINGS valid ball detections, transform each to base_link,
        then return the median XYZ — much more stable than a single reading.
        """
        self.get_logger().info(
            f"Collecting {N_READINGS} readings for stable pose estimate..."
        )
        self._raw_readings.clear()
        base_points = []
        deadline = self.get_clock().now() + Duration(seconds=MAX_WAIT_SEC)

        while len(base_points) < N_READINGS:
            if self.get_clock().now() > deadline:
                self.get_logger().error(
                    f"Timeout: only got {len(base_points)}/{N_READINGS} valid readings"
                )
                return None

            rclpy.spin_once(self, timeout_sec=0.05)

            while self._raw_readings:
                raw = self._raw_readings.pop(0)
                transformed = self._transform_to_base(raw, retries=2)
                if transformed is None:
                    continue

                bx = transformed.point.x
                by = transformed.point.y
                bz = transformed.point.z

                # Sanity check — reject readings outside the workspace
                if not (MIN_X <= bx <= MAX_X and
                        MIN_Y <= by <= MAX_Y and
                        MIN_Z <= bz <= MAX_Z):
                    self.get_logger().debug(
                        f"Rejected out-of-workspace reading: "
                        f"({bx:.3f}, {by:.3f}, {bz:.3f})"
                    )
                    continue

                base_points.append((bx, by, bz))
                self.get_logger().info(
                    f"  Reading {len(base_points)}/{N_READINGS}: "
                    f"({bx:.3f}, {by:.3f}, {bz:.3f}) m"
                )

        # Use median to reject outliers
        xs = [p[0] for p in base_points]
        ys = [p[1] for p in base_points]
        zs = [p[2] for p in base_points]

        bx = float(np.median(xs))
        by = float(np.median(ys))
        bz = float(np.median(zs))

        spread = max(np.std(xs), np.std(ys), np.std(zs))
        self.get_logger().info(
            f"Stable ball position: ({bx:.4f}, {by:.4f}, {bz:.4f}) m  "
            f"spread={spread:.4f} m"
        )

        if spread > 0.05:
            self.get_logger().warn(
                f"High spread ({spread:.3f}m) — camera or ball may be moving. "
                f"Results may be inaccurate."
            )

        return bx, by, bz

    # ── IK solver ─────────────────────────────────────────────────────────────

    def _compute_ik(self, x, y, z):
        base_angle = math.atan2(y, x)
        base_angle = clamp(base_angle, BASE_LIMIT)

        r_total = math.sqrt(x**2 + y**2)
        r = max(0.01, r_total - GRIPPER_LENGTH)

        try:
            shoulder, elbow = planar_ik(r, z)
        except ValueError as e:
            self.get_logger().error(f"IK failed: {e}")
            return None

        wrist = clamp(-(shoulder + elbow), WRIST_LIMIT)
        shoulder = clamp(shoulder, SHOULDER_LIMIT)
        elbow    = clamp(elbow,    ELBOW_LIMIT)

        return [base_angle, shoulder, elbow, wrist]

    # ── Main sequence ─────────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info("=" * 55)
        self.get_logger().info("  Vision pick and place — starting")
        self.get_logger().info("=" * 55)

        # Step 1 — Home
        self.get_logger().info("Step 1 — Home + open gripper")
        self._send_arm(POSE_HOME)
        self._send_gripper(GRIPPER_OPEN)

        # Step 2 — Scan pose
        self.get_logger().info("Step 2 — Scan pose")
        self._send_arm(POSE_SCAN)
        time.sleep(1.0)   # let arm settle fully before reading camera

        # Step 3 — Detect ball with averaging
        self.get_logger().info("Step 3 — Detecting green ball (averaged)")
        result = self._get_stable_ball_pose()
        if result is None:
            self.get_logger().error("Detection failed — returning home")
            self._send_arm(POSE_HOME)
            return
        bx, by, bz = result

        # Subtract ball radius so we target the ball's top surface
        pick_z = bz - BALL_RADIUS

        # Step 4 — Plan IK
        self.get_logger().info(
            f"Step 4 — Planning IK for ball at ({bx:.3f}, {by:.3f}, {pick_z:.3f})"
        )
        pre_pick = self._compute_ik(bx, by, pick_z + PICK_APPROACH_OFFSET)
        pick     = self._compute_ik(bx, by, pick_z)

        if pre_pick is None or pick is None:
            self.get_logger().error("IK failed — check workspace limits and link lengths")
            self._send_arm(POSE_HOME)
            return

        # Step 5 — Pre-pick approach
        self.get_logger().info("Step 5 — Pre-pick approach")
        self._send_arm(pre_pick)

        # Step 6 — Lower to ball
        self.get_logger().info("Step 6 — Lower to ball")
        self._send_arm(pick, duration=T_PICK)
        time.sleep(0.3)

        # Step 7 — Grasp
        self.get_logger().info("Step 7 — Grasp")
        self._send_gripper(GRIPPER_CLOSED)
        time.sleep(0.5)

        # Step 8 — Lift
        self.get_logger().info("Step 8 — Lift")
        self._send_arm(pre_pick)

        # Step 9 — Place
        self.get_logger().info("Step 9 — Move to place location")
        self._send_arm(POSE_PLACE)

        # Step 10 — Release
        self.get_logger().info("Step 10 — Release")
        self._send_gripper(GRIPPER_OPEN)
        time.sleep(0.3)

        # Step 11 — Home
        self.get_logger().info("Step 11 — Return home")
        self._send_arm(POSE_HOME)

        self.get_logger().info("=" * 55)
        self.get_logger().info("  Complete!")
        self.get_logger().info("=" * 55)


def main(args=None):
    rclpy.init(args=args)
    node = VisionPickPlace()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted — returning home")
        node._send_arm(POSE_HOME)
        node._send_gripper(GRIPPER_OPEN)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()