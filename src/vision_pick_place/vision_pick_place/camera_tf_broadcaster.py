#!/usr/bin/env python3
"""
camera_tf_broadcaster.py  (fixed)
────────────────────────────────────────────────────────────────────────────
Publishes the static TF that connects the robot arm TF tree to the
RealSense camera TF tree.

The RealSense driver (realsense2_camera) publishes its own internal TF tree:
    camera_link → camera_depth_frame → camera_color_frame
                                     → camera_color_optical_frame

We need to connect:
    gripper  →  camera_link

This single transform bridges the two trees so tf2 can resolve:
    base_link → ... → gripper → camera_link → camera_color_optical_frame

Measure TX, TY, TZ by placing the arm at home position and measuring
from the gripper link origin (wrist joint centre) to the camera body.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math


# ── Parent = your URDF gripper link name ─────────────────────────────────────
# Must exactly match the link name in arm_robot_pkg.urdf
PARENT_FRAME = "gripper"

# ── Child = the root frame published by realsense2_camera ────────────────────
# The RealSense driver publishes camera_link as the root of its TF tree.
# This bridges the robot tree to the camera tree.
CHILD_FRAME = "camera_link"

# ── Translation: camera body centre relative to gripper link origin (metres) ─
# Measure these on your physical arm with a ruler.
# Positive X = forward (same direction gripper faces)
# Positive Y = left
# Positive Z = up
TX = 0.04    # adjust to your mount
TY = 0.00
TZ = 0.015

# ── Rotation: how the camera is oriented relative to the gripper ─────────────
# If camera is mounted facing the same direction as the gripper and upright:
ROLL  = 0.0
PITCH = 0.0
YAW   = 0.0

# If camera is mounted facing forward but rotated 90° sideways, change YAW.
# Use rqt_tf_tree or view_frames to visually verify after launching.


def rpy_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll  / 2);  sr = math.sin(roll  / 2)
    cp = math.cos(pitch / 2);  sp = math.sin(pitch / 2)
    cy = math.cos(yaw   / 2);  sy = math.sin(yaw   / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class CameraTFBroadcaster(Node):

    def __init__(self):
        super().__init__("camera_tf_broadcaster")

        self._br = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = PARENT_FRAME
        t.child_frame_id  = CHILD_FRAME

        t.transform.translation.x = TX
        t.transform.translation.y = TY
        t.transform.translation.z = TZ

        qx, qy, qz, qw = rpy_to_quaternion(ROLL, PITCH, YAW)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self._br.sendTransform(t)

        self.get_logger().info(
            f"Static TF: {PARENT_FRAME} → {CHILD_FRAME}  "
            f"xyz=({TX},{TY},{TZ})  rpy=({ROLL:.2f},{PITCH:.2f},{YAW:.2f})"
        )

        # Verify the full chain is resolvable after a short delay
        self.create_timer(2.0, self._verify_chain)

    def _verify_chain(self):
        """Check that base_link → camera_color_optical_frame is now connected."""
        import tf2_ros
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf, self)
        import time; time.sleep(0.5)
        try:
            buf.lookup_transform(
                "base_link",
                "camera_color_optical_frame",
                rclpy.time.Time(),
            )
            self.get_logger().info(
                "TF chain verified: base_link → camera_color_optical_frame OK"
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF chain not yet complete: {e}\n"
                f"Check that robot_state_publisher and realsense2_camera are running."
            )


def main(args=None):
    rclpy.init(args=args)
    node = CameraTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()