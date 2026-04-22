#!/usr/bin/env python3
"""
green_ball_detector.py
────────────────────────────────────────────────────────────────────────────
Detects a light-green ball in the RealSense D435 colour+depth stream and
publishes its 3D position in the camera optical frame.

Subscriptions:
  /camera/camera/color/image_raw          sensor_msgs/Image
  /camera/camera/depth/image_rect_raw     sensor_msgs/Image
  /camera/camera/color/camera_info        sensor_msgs/CameraInfo

Publications:
  /detected_ball_pose   geometry_msgs/PointStamped  (camera optical frame)
  /ball_debug_image     sensor_msgs/Image            (annotated, for RViz)

How it works:
  1. Convert RGB frame to HSV colour space
  2. Threshold to isolate light-green pixels (tunable HSV range below)
  3. Find the largest contour — that is the ball
  4. Get the centroid pixel (u, v)
  5. Look up the depth value at (u, v) from the aligned depth image
  6. Back-project (u, v, Z) → 3D point using camera intrinsics
  7. Publish as PointStamped in the camera frame

Tuning HSV:
  Run with debug:=true, watch /ball_debug_image in RViz or rqt_image_view
  Adjust H_LOW/H_HIGH/S_LOW/V_LOW below until the ball mask is clean.
  Light green in HSV: H ≈ 40–80, S ≈ 50–255, V ≈ 50–255
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import numpy as np
import cv2

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber


# ── HSV colour range for light-green ball ─────────────────────────────────────
# Tune these values using the debug image in rqt_image_view
# H: 0-179,  S: 0-255,  V: 0-255  in OpenCV HSV space
H_LOW,  H_HIGH  =  35,  85    # hue — green range
S_LOW,  S_HIGH  =  50, 255    # saturation — avoids grey/white
V_LOW,  V_HIGH  =  50, 255    # value — avoids very dark pixels

# Minimum ball size in pixels (rejects noise)
MIN_BALL_AREA = 200   # pixels²  — increase if detecting noise
MIN_BALL_RADIUS = 8   # pixels   — minimum circle radius

# Depth reading averaging kernel (samples a small area around centroid)
DEPTH_KERNEL_RADIUS = 3   # pixels


class GreenBallDetector(Node):

    def __init__(self):
        super().__init__("green_ball_detector")

        self.declare_parameter("debug", True)
        self._debug = self.get_parameter("debug").value

        self._bridge = CvBridge()
        self._camera_info = None

        # ── Synchronised colour + depth subscribers ───────────────────────────
        self._color_sub = Subscriber(
            self, Image, "/camera/camera/color/image_raw"
        )
        self._depth_sub = Subscriber(
            self, Image, "/camera/camera/depth/image_rect_raw"
        )

        self._sync = ApproximateTimeSynchronizer(
            [self._color_sub, self._depth_sub],
            queue_size=10,
            slop=0.05,   # 50 ms sync tolerance
        )
        self._sync.registerCallback(self._on_frames)

        # Camera intrinsics (needed for back-projection)
        self._info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self._on_camera_info,
            10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._pose_pub = self.create_publisher(
            PointStamped, "/detected_ball_pose", 10
        )
        self._debug_pub = self.create_publisher(
            Image, "/ball_debug_image", 10
        )

        self.get_logger().info(
            f"Green ball detector ready  "
            f"HSV range H=[{H_LOW},{H_HIGH}] S=[{S_LOW},{S_HIGH}] "
            f"V=[{V_LOW},{V_HIGH}]"
        )

    # ── Camera intrinsics callback ────────────────────────────────────────────

    def _on_camera_info(self, msg: CameraInfo):
        if self._camera_info is None:
            self._camera_info = msg
            self.get_logger().info(
                f"Camera intrinsics received  "
                f"fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} "
                f"cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}"
            )

    # ── Main detection callback ───────────────────────────────────────────────

    def _on_frames(self, color_msg: Image, depth_msg: Image):
        if self._camera_info is None:
            return   # wait for intrinsics

        # Convert ROS images to OpenCV arrays
        color_bgr  = self._bridge.imgmsg_to_cv2(color_msg,  "bgr8")
        depth_raw  = self._bridge.imgmsg_to_cv2(depth_msg,  "passthrough")

        # depth_raw is uint16 in millimetres for RealSense aligned depth
        depth_m = depth_raw.astype(np.float32) / 1000.0  # → metres

        h, w = color_bgr.shape[:2]

        # ── Colour segmentation ───────────────────────────────────────────────
        hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)

        lower = np.array([H_LOW,  S_LOW,  V_LOW],  dtype=np.uint8)
        upper = np.array([H_HIGH, S_HIGH, V_HIGH], dtype=np.uint8)
        mask  = cv2.inRange(hsv, lower, upper)

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask   = cv2.erode(mask,  kernel, iterations=1)
        mask   = cv2.dilate(mask, kernel, iterations=2)

        # ── Find largest contour (the ball) ───────────────────────────────────
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        debug_img = color_bgr.copy() if self._debug else None
        ball_found = False

        if contours:
            # Pick the largest contour by area
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area >= MIN_BALL_AREA:
                # Fit a minimum enclosing circle
                (cx, cy), radius = cv2.minEnclosingCircle(largest)
                cx, cy = int(cx), int(cy)

                if radius >= MIN_BALL_RADIUS:
                    # ── Depth lookup at centroid ──────────────────────────────
                    r = DEPTH_KERNEL_RADIUS
                    u0 = max(0, cx - r);  u1 = min(w, cx + r + 1)
                    v0 = max(0, cy - r);  v1 = min(h, cy + r + 1)
                    patch = depth_m[v0:v1, u0:u1]

                    # Use median of non-zero depth values (rejects holes)
                    valid = patch[patch > 0.05]   # ignore < 5 cm (bad readings)
                    if len(valid) == 0:
                        self.get_logger().warn(
                            "Ball detected in colour but no valid depth at centroid"
                        )
                    else:
                        Z = float(np.median(valid))

                        # ── Back-project pixel → 3D ───────────────────────────
                        fx = self._camera_info.k[0]
                        fy = self._camera_info.k[4]
                        ppx = self._camera_info.k[2]
                        ppy = self._camera_info.k[5]

                        X = (cx - ppx) * Z / fx
                        Y = (cy - ppy) * Z / fy
                        # Z is already in metres, positive = forward from camera

                        # ── Publish 3D position ───────────────────────────────
                        pose = PointStamped()
                        pose.header = color_msg.header
                        pose.point.x = X
                        pose.point.y = Y
                        pose.point.z = Z
                        self._pose_pub.publish(pose)

                        ball_found = True

                        self.get_logger().info(
                            f"Ball detected  pixel=({cx},{cy})  "
                            f"3D=({X:.3f}, {Y:.3f}, {Z:.3f}) m  "
                            f"r={radius:.0f}px"
                        )

                    # ── Debug annotation ──────────────────────────────────────
                    if self._debug and debug_img is not None:
                        col = (0, 255, 0) if ball_found else (0, 165, 255)
                        cv2.circle(debug_img, (cx, cy),
                                   int(radius), col, 2)
                        cv2.circle(debug_img, (cx, cy), 4, (0, 0, 255), -1)
                        if ball_found:
                            label = f"Z={Z:.2f}m"
                            cv2.putText(debug_img, label,
                                        (cx + 10, cy - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.6, (0, 255, 0), 2)

        if self._debug and debug_img is not None:
            # Draw HSV mask as overlay in top-right corner
            mask_small = cv2.resize(
                cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR),
                (w // 4, h // 4)
            )
            debug_img[0:h // 4, w - w // 4:w] = mask_small
            self._debug_pub.publish(
                self._bridge.cv2_to_imgmsg(debug_img, "bgr8")
            )


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GreenBallDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()