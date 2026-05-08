#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
from enum import Enum

import numpy as np
import rospy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PointStamped
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

import tf2_ros
import tf2_geometry_msgs

from yolo_tracking.msg import YoloDetection


class RobotState(Enum):
    SEARCHING = 0
    ALIGNING = 1
    APPROACHING = 2
    FOLLOWING = 3


class YoloFollowerNode:
    def __init__(self):
        rospy.init_node("yolo_follower_node", anonymous=False)

        self.det_topic = rospy.get_param("~det_topic", "/yolo_detection")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/rgb/camera_info")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.target_class_topic = rospy.get_param("~target_class_topic", "/target_class_cmd")

        self.source_frame = rospy.get_param("~source_frame", "camera_rgb_optical_frame")
        self.target_frame = rospy.get_param("~target_frame", "base_link")

        self.target_class = rospy.get_param("~target_class", "person")
        self.use_detection_class_directly = bool(
            rospy.get_param("~use_detection_class_directly", True)
        )

        self.desired_distance = float(rospy.get_param("~desired_distance", 0.8))
        self.distance_threshold = float(rospy.get_param("~distance_threshold", 0.12))
        self.align_threshold = float(rospy.get_param("~align_threshold", 0.15))

        self.angular_kp = float(rospy.get_param("~angular_kp", 1.2))
        self.linear_kp = float(rospy.get_param("~linear_kp", 0.45))

        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.20))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 0.55))
        self.search_angular_speed = float(rospy.get_param("~search_angular_speed", 0.22))

        self.max_lost_frames = int(rospy.get_param("~max_lost_frames", 10))
        self.depth_roi_radius = int(rospy.get_param("~depth_roi_radius", 8))

        self.min_valid_depth = float(rospy.get_param("~min_valid_depth", 0.15))
        self.max_valid_depth = float(rospy.get_param("~max_valid_depth", 5.0))

        self.position_filter_alpha = float(rospy.get_param("~position_filter_alpha", 0.45))
        self.velocity_filter_beta = float(rospy.get_param("~velocity_filter_beta", 0.35))

        self.allow_backward = bool(rospy.get_param("~allow_backward", False))
        self.control_rate_hz = float(rospy.get_param("~control_rate_hz", 15.0))

        self.detection_timeout = float(rospy.get_param("~detection_timeout", 0.8))
        self.enable_search_rotation = bool(rospy.get_param("~enable_search_rotation", True))

        self.bridge = CvBridge()
        self.data_lock = threading.Lock()

        self.depth_image = None
        self.depth_encoding = None
        self.depth_stamp = None

        self.camera_matrix = None
        self.camera_info_frame = None

        self.latest_detection = None
        self.latest_detection_receive_time = None

        self.current_state = RobotState.SEARCHING
        self.target_lost_count = self.max_lost_frames

        self.filtered_target_x = None
        self.filtered_target_y = None
        self.filtered_target_z = None

        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0

        self.last_seen_side = 1.0
        self.current_detected_class = self.target_class

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)

        self.det_sub = rospy.Subscriber(
            self.det_topic,
            YoloDetection,
            self.detection_callback,
            queue_size=10
        )

        self.depth_sub = rospy.Subscriber(
            self.depth_topic,
            Image,
            self.depth_callback,
            queue_size=1,
            buff_size=2**24
        )

        self.info_sub = rospy.Subscriber(
            self.camera_info_topic,
            CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )

        self.target_class_sub = rospy.Subscriber(
            self.target_class_topic,
            String,
            self.target_class_callback,
            queue_size=10
        )

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("=================================================")
        rospy.loginfo("YOLO Follower Node Started")
        rospy.loginfo("Detection topic               : %s", self.det_topic)
        rospy.loginfo("Depth topic                   : %s", self.depth_topic)
        rospy.loginfo("CameraInfo topic              : %s", self.camera_info_topic)
        rospy.loginfo("CmdVel topic                  : %s", self.cmd_vel_topic)
        rospy.loginfo("Target class topic            : %s", self.target_class_topic)
        rospy.loginfo("Initial target class          : %s", self.target_class)
        rospy.loginfo("Use detection class directly  : %s", self.use_detection_class_directly)
        rospy.loginfo("Source frame                  : %s", self.source_frame)
        rospy.loginfo("Target frame                  : %s", self.target_frame)
        rospy.loginfo("Desired distance              : %.2f m", self.desired_distance)
        rospy.loginfo("=================================================")

    def target_class_callback(self, msg):
        new_target = msg.data.strip()
        if not new_target:
            return

        if new_target != self.target_class:
            rospy.loginfo("Follower target_class updated: %s -> %s", self.target_class, new_target)
            self.target_class = new_target

            if not self.use_detection_class_directly:
                with self.data_lock:
                    self.latest_detection = None
                    self.latest_detection_receive_time = None

                self.current_state = RobotState.SEARCHING
                self.target_lost_count = self.max_lost_frames
                self.reset_target_filter()
                self.stop_robot()

    def detection_callback(self, msg):
        with self.data_lock:
            self.latest_detection = msg
            self.latest_detection_receive_time = rospy.Time.now()

        if msg.detected:
            self.current_detected_class = msg.class_name

    def depth_callback(self, msg):
        try:
            if msg.encoding == "16UC1":
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                depth = depth.astype(np.float32) / 1000.0
            elif msg.encoding == "32FC1":
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                depth = depth.astype(np.float32)
            else:
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                depth = depth.astype(np.float32)

            with self.data_lock:
                self.depth_image = depth
                self.depth_encoding = msg.encoding
                self.depth_stamp = msg.header.stamp

        except CvBridgeError as e:
            rospy.logerr_throttle(1.0, "Depth image conversion failed: %s", e)
        except Exception as e:
            rospy.logerr_throttle(1.0, "Depth callback error: %s", e)

    def camera_info_callback(self, msg):
        with self.data_lock:
            self.camera_matrix = np.array(msg.K, dtype=np.float32).reshape(3, 3)
            self.camera_info_frame = msg.header.frame_id

    def get_depth_median(self, depth_image, u, v, radius):
        h, w = depth_image.shape[:2]

        if u < 0 or u >= w or v < 0 or v >= h:
            rospy.logwarn_throttle(
                1.0,
                "Pixel out of depth image range: u=%d v=%d depth_size=(%d,%d)",
                u, v, w, h
            )
            return None

        x1 = max(0, u - radius)
        x2 = min(w, u + radius + 1)
        y1 = max(0, v - radius)
        y2 = min(h, v + radius + 1)

        roi = depth_image[y1:y2, x1:x2]
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid > self.min_valid_depth) & (valid < self.max_valid_depth)]

        if valid.size == 0:
            return None

        return float(np.median(valid))

    def pixel_to_camera_point(self, u, v, depth_m, camera_matrix):
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        if fx == 0 or fy == 0:
            return None

        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m

        return x, y, z

    def transform_to_base(self, x, y, z):
        point_camera = PointStamped()
        point_camera.header.stamp = rospy.Time(0)
        point_camera.header.frame_id = self.source_frame
        point_camera.point.x = x
        point_camera.point.y = y
        point_camera.point.z = z

        try:
            point_base = self.tf_buffer.transform(
                point_camera,
                self.target_frame,
                rospy.Duration(0.2)
            )
            return point_base.point.x, point_base.point.y, point_base.point.z
        except Exception as e:
            rospy.logwarn_throttle(1.0, "TF transform failed: %s", e)
            return None

    def filter_position(self, x, y, z):
        if self.filtered_target_x is None:
            self.filtered_target_x = x
            self.filtered_target_y = y
            self.filtered_target_z = z
        else:
            a = self.position_filter_alpha
            self.filtered_target_x = a * x + (1.0 - a) * self.filtered_target_x
            self.filtered_target_y = a * y + (1.0 - a) * self.filtered_target_y
            self.filtered_target_z = a * z + (1.0 - a) * self.filtered_target_z

        return (
            self.filtered_target_x,
            self.filtered_target_y,
            self.filtered_target_z
        )

    def smooth_cmd(self, linear, angular):
        b = self.velocity_filter_beta

        smoothed_linear = b * linear + (1.0 - b) * self.last_cmd_linear
        smoothed_angular = b * angular + (1.0 - b) * self.last_cmd_angular

        self.last_cmd_linear = smoothed_linear
        self.last_cmd_angular = smoothed_angular

        return smoothed_linear, smoothed_angular

    @staticmethod
    def clip(value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0

    def publish_cmd(self, linear, angular):
        linear, angular = self.smooth_cmd(linear, angular)

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

        rospy.loginfo_throttle(
            1.0,
            "[Control] State=%s linear=%.3f angular=%.3f",
            self.current_state.name,
            linear,
            angular
        )

    def reset_target_filter(self):
        self.filtered_target_x = None
        self.filtered_target_y = None
        self.filtered_target_z = None

    def get_current_target_position(self):
        with self.data_lock:
            det = self.latest_detection
            det_receive_time = self.latest_detection_receive_time
            depth = None if self.depth_image is None else self.depth_image.copy()
            K = None if self.camera_matrix is None else self.camera_matrix.copy()

        if det is None:
            rospy.logwarn_throttle(2.0, "No detection message received yet")
            return None

        if det_receive_time is not None:
            dt = (rospy.Time.now() - det_receive_time).to_sec()
            if dt > self.detection_timeout:
                rospy.logwarn_throttle(1.0, "Detection timeout: %.2f s", dt)
                return None

        if depth is None:
            rospy.logwarn_throttle(2.0, "No depth image received yet")
            return None

        if K is None:
            rospy.logwarn_throttle(2.0, "No camera info received yet")
            return None

        if not det.detected:
            return None

        if not self.use_detection_class_directly:
            if det.class_name != self.target_class:
                rospy.logwarn_throttle(
                    1.0,
                    "Detection class mismatch: got=%s expected=%s",
                    det.class_name,
                    self.target_class
                )
                return None

        u = int(det.center_x)
        v = int(det.center_y)

        depth_m = self.get_depth_median(depth, u, v, self.depth_roi_radius)
        if depth_m is None:
            rospy.logwarn_throttle(
                1.0,
                "No valid depth around target center: u=%d v=%d",
                u,
                v
            )
            return None

        cam_point = self.pixel_to_camera_point(u, v, depth_m, K)
        if cam_point is None:
            rospy.logwarn_throttle(1.0, "Invalid camera intrinsic matrix")
            return None

        cam_x, cam_y, cam_z = cam_point

        base_point = self.transform_to_base(cam_x, cam_y, cam_z)
        if base_point is None:
            return None

        bx, by, bz = base_point
        bx, by, bz = self.filter_position(bx, by, bz)

        rospy.loginfo_throttle(
            1.0,
            "[Target3D] class=%s bbox=[%d,%d,%d,%d] center=(%d,%d) conf=%.2f base_link=(%.2f, %.2f, %.2f)",
            det.class_name,
            det.x1, det.y1, det.x2, det.y2,
            u, v,
            det.confidence,
            bx, by, bz
        )

        return bx, by, bz

    def update_state_and_control(self):
        target = self.get_current_target_position()

        if target is None:
            self.target_lost_count += 1

            if self.target_lost_count >= self.max_lost_frames:
                self.current_state = RobotState.SEARCHING
                self.reset_target_filter()

                if self.enable_search_rotation:
                    angular = self.search_angular_speed * self.last_seen_side
                    self.publish_cmd(0.0, angular)
                else:
                    self.publish_cmd(0.0, 0.0)
            else:
                self.publish_cmd(0.0, 0.0)

            return

        self.target_lost_count = 0

        x, y, z = target

        if y > 0.02:
            self.last_seen_side = 1.0
        elif y < -0.02:
            self.last_seen_side = -1.0

        if x <= 0.05:
            rospy.logwarn_throttle(1.0, "Target x is too small or behind robot: x=%.2f", x)
            self.current_state = RobotState.SEARCHING
            self.publish_cmd(0.0, self.search_angular_speed * self.last_seen_side)
            return

        angle_error = math.atan2(y, x)
        distance_error = x - self.desired_distance

        if abs(angle_error) > self.align_threshold:
            self.current_state = RobotState.ALIGNING
        elif abs(distance_error) > self.distance_threshold:
            self.current_state = RobotState.APPROACHING
        else:
            self.current_state = RobotState.FOLLOWING

        if self.current_state == RobotState.ALIGNING:
            linear = 0.0
            angular = self.angular_kp * angle_error

        elif self.current_state == RobotState.APPROACHING:
            linear = self.linear_kp * distance_error
            angular = self.angular_kp * angle_error

        elif self.current_state == RobotState.FOLLOWING:
            angular = self.angular_kp * angle_error
            if abs(distance_error) < self.distance_threshold:
                linear = 0.0
            else:
                linear = self.linear_kp * distance_error

        else:
            linear = 0.0
            angular = self.search_angular_speed * self.last_seen_side

        if not self.allow_backward:
            linear = max(0.0, linear)

        linear = self.clip(linear, -self.max_linear_speed, self.max_linear_speed)
        angular = self.clip(angular, -self.max_angular_speed, self.max_angular_speed)

        rospy.loginfo_throttle(
            1.0,
            "[State] %s tracked_class=%s x=%.2f y=%.2f angle=%.2f dist_err=%.2f",
            self.current_state.name,
            self.current_detected_class,
            x,
            y,
            angle_error,
            distance_error
        )

        self.publish_cmd(linear, angular)

    def spin(self):
        rate = rospy.Rate(self.control_rate_hz)

        rospy.loginfo("YOLO Follower control loop started")

        while not rospy.is_shutdown():
            self.update_state_and_control()
            rate.sleep()

    def on_shutdown(self):
        rospy.loginfo("Shutting down YOLO follower node, stopping robot...")
        self.stop_robot()


if __name__ == "__main__":
    try:
        node = YoloFollowerNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
