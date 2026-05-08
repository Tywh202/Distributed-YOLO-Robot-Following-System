#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import socket
import threading

import rospy
from std_msgs.msg import Header
from yolo_tracking.msg import YoloDetection


class YoloDetectionBridgeNode:
    def __init__(self):
        rospy.init_node("yolo_detection_bridge_node", anonymous=False)

        self.host = rospy.get_param("~host", "0.0.0.0")
        self.port = int(rospy.get_param("~port", 5005))
        self.det_topic = rospy.get_param("~det_topic", "/yolo_detection")

        self.pub = rospy.Publisher(self.det_topic, YoloDetection, queue_size=20)

        self.server_socket = None
        self.client_threads = []
        self.running = True

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("==============================================")
        rospy.loginfo("YOLO Detection Bridge Node Started")
        rospy.loginfo("Listen host   : %s", self.host)
        rospy.loginfo("Listen port   : %d", self.port)
        rospy.loginfo("Publish topic : %s", self.det_topic)
        rospy.loginfo("==============================================")

    def build_msg(self, data_dict):
        msg = YoloDetection()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "tcp_bridge"

        msg.detected = bool(data_dict.get("detected", False))
        msg.class_name = str(data_dict.get("class_name", ""))
        msg.confidence = float(data_dict.get("confidence", 0.0))
        msg.x1 = int(data_dict.get("x1", 0))
        msg.y1 = int(data_dict.get("y1", 0))
        msg.x2 = int(data_dict.get("x2", 0))
        msg.y2 = int(data_dict.get("y2", 0))
        msg.center_x = int(data_dict.get("center_x", 0))
        msg.center_y = int(data_dict.get("center_y", 0))

        return msg

    def handle_client(self, conn, addr):
        rospy.loginfo("Client connected: %s:%d", addr[0], addr[1])

        buffer = ""
        try:
            while self.running and not rospy.is_shutdown():
                data = conn.recv(4096)
                if not data:
                    break

                buffer += data.decode("utf-8")

                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    try:
                        data_dict = json.loads(line)
                        msg = self.build_msg(data_dict)
                        self.pub.publish(msg)

                        rospy.loginfo_throttle(
                            1.0,
                            "[Bridge] detected=%s class=%s conf=%.2f bbox=[%d,%d,%d,%d] center=(%d,%d)",
                            msg.detected,
                            msg.class_name,
                            msg.confidence,
                            msg.x1, msg.y1, msg.x2, msg.y2,
                            msg.center_x, msg.center_y
                        )

                    except json.JSONDecodeError as e:
                        rospy.logwarn("JSON decode error: %s, line=%s", e, line)
                    except Exception as e:
                        rospy.logwarn("Failed to process message: %s", e)

        except Exception as e:
            rospy.logwarn("Client connection error from %s:%d -> %s", addr[0], addr[1], e)

        finally:
            try:
                conn.close()
            except Exception:
                pass
            rospy.loginfo("Client disconnected: %s:%d", addr[0], addr[1])

    def spin(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        self.server_socket.settimeout(1.0)

        rospy.loginfo("TCP server listening on %s:%d ...", self.host, self.port)

        while not rospy.is_shutdown() and self.running:
            try:
                conn, addr = self.server_socket.accept()
                t = threading.Thread(target=self.handle_client, args=(conn, addr), daemon=True)
                t.start()
                self.client_threads.append(t)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    rospy.logwarn("Accept failed: %s", e)

    def on_shutdown(self):
        self.running = False
        rospy.loginfo("Shutting down bridge node...")
        try:
            if self.server_socket is not None:
                self.server_socket.close()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        node = YoloDetectionBridgeNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
