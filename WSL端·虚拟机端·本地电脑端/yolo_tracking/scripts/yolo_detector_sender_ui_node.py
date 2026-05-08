#!/usr/local/bin/python3.10
# -*- coding: utf-8 -*-

import json
import math
import socket
import threading
import traceback

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import tkinter as tk
from tkinter import messagebox

try:
    from ultralytics import YOLO
except Exception as e:
    raise RuntimeError("无法导入 ultralytics，请先安装。错误: {}".format(e))


class YoloDetectorSenderUINode:
    def __init__(self):
        rospy.init_node("yolo_detector_sender_ui_node", anonymous=False)

        # ROS / YOLO 参数
        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_raw")
        self.model_path = rospy.get_param("~model_path", "/home/fish/catkin_ws/src/yolo_tracking/models/yolo26s.pt")
        self.target_class = rospy.get_param("~target_class", "person")
        self.conf_thres = float(rospy.get_param("~conf_thres", 0.45))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.device = rospy.get_param("~device", "0")
        self.show_window = bool(rospy.get_param("~show_window", True))
        self.max_jump_dist = float(rospy.get_param("~max_jump_dist", 120.0))
        self.send_no_detection = bool(rospy.get_param("~send_no_detection", True))

        # TCP 参数
        self.server_ip = rospy.get_param("~server_ip", "10.31.108.35")
        self.server_port = int(rospy.get_param("~server_port", 5005))
        self.reconnect_interval = float(rospy.get_param("~reconnect_interval", 2.0))

        # UI 参数
        self.supported_classes = rospy.get_param(
            "~supported_classes",
            ["person", "chair", "bottle", "backpack", "cup", "laptop", "book", "cell phone"]
        )

        # 数据
        self.bridge = CvBridge()
        self.img_lock = threading.Lock()

        self.latest_bgr = None
        self.latest_header = None

        self.last_target_center = None
        self.last_detected = False

        self.sock = None
        self.connected = False
        self.sock_lock = threading.Lock()

        self.ui_lock = threading.Lock()
        self.current_target_var = None
        self.log_text = None
        self.root = None

        # 订阅图像
        self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self.rgb_callback, queue_size=1, buff_size=2**24)

        # 加载模型
        rospy.loginfo("正在加载 YOLO 模型: %s", self.model_path)
        self.model = YOLO(self.model_path)
        self.class_names = self.model.names
        rospy.loginfo("YOLO 模型加载完成")
        rospy.loginfo("初始目标类别: %s", self.target_class)

        # UI
        self.init_ui()

        rospy.on_shutdown(self.on_shutdown)

    # =========================================================
    # UI
    # =========================================================
    def init_ui(self):
        self.root = tk.Tk()
        self.root.title("YOLO Detector Sender UI")
        self.root.geometry("560x360")
        self.root.resizable(False, False)

        title = tk.Label(self.root, text="YOLO Tracking Target Selector", font=("Arial", 16, "bold"))
        title.pack(pady=10)

        info = tk.Label(self.root, text="选择要跟踪的类别（直接作用于检测节点内部）", font=("Arial", 11))
        info.pack(pady=4)

        current_frame = tk.Frame(self.root)
        current_frame.pack(pady=6)

        tk.Label(current_frame, text="当前目标类别：", font=("Arial", 12)).pack(side=tk.LEFT)

        self.current_target_var = tk.StringVar(value=self.target_class)
        tk.Label(current_frame, textvariable=self.current_target_var, font=("Arial", 12, "bold"), fg="blue").pack(side=tk.LEFT)

        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=12)

        cols = 4
        for idx, cls_name in enumerate(self.supported_classes):
            btn = tk.Button(
                button_frame,
                text=cls_name,
                width=12,
                height=2,
                command=lambda c=cls_name: self.on_select_target(c)
            )
            r = idx // cols
            c = idx % cols
            btn.grid(row=r, column=c, padx=6, pady=6)

        self.log_text = tk.Text(self.root, height=8, width=64)
        self.log_text.pack(pady=10)
        self.append_log("UI started.")
        self.append_log("Default target: {}".format(self.target_class))
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def append_log(self, text):
        if self.log_text is None:
            return
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, text + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def on_select_target(self, target_name):
        with self.ui_lock:
            old_target = self.target_class
            self.target_class = target_name
            self.last_target_center = None
            self.last_detected = False
            self.current_target_var.set(target_name)

        rospy.loginfo("目标类别切换: %s -> %s", old_target, target_name)
        self.append_log("Target switched: {} -> {}".format(old_target, target_name))

    def on_close(self):
        if messagebox.askokcancel("Quit", "关闭 UI 并结束检测节点？"):
            rospy.signal_shutdown("UI closed by user")

    def update_ui(self):
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            pass

    # =========================================================
    # 图像与检测
    # =========================================================
    def rgb_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.img_lock:
                self.latest_bgr = cv_image
                self.latest_header = msg.header
        except CvBridgeError as e:
            rospy.logerr_throttle(1.0, "RGB 图像转换失败: %s", e)

    @staticmethod
    def bbox_area(x1, y1, x2, y2):
        return max(0, x2 - x1) * max(0, y2 - y1)

    @staticmethod
    def center_distance(c1, c2):
        return math.sqrt((c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2)

    def select_target(self, candidates):
        if len(candidates) == 0:
            return None

        if self.last_target_center is not None:
            candidates_sorted = sorted(
                candidates,
                key=lambda d: self.center_distance((d["cx"], d["cy"]), self.last_target_center)
            )
            best = candidates_sorted[0]
            dist = self.center_distance((best["cx"], best["cy"]), self.last_target_center)

            if dist <= self.max_jump_dist:
                return best

            rospy.logwarn_throttle(1.0, "目标跳变较大 dist=%.1f，退化为面积最大目标策略", dist)

        best = max(candidates, key=lambda d: d["area"])
        return best

    def run_inference(self, frame_bgr):
        with self.ui_lock:
            current_target = self.target_class

        results = self.model.predict(
            source=frame_bgr,
            conf=self.conf_thres,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False
        )

        candidates = []
        if results is None or len(results) == 0:
            return candidates, current_target

        result = results[0]
        boxes = result.boxes

        if boxes is None:
            return candidates, current_target

        for box in boxes:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            xyxy = box.xyxy[0].cpu().numpy().astype(int)
            x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])

            class_name = self.class_names.get(cls_id, str(cls_id))

            if class_name != current_target:
                continue

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            area = self.bbox_area(x1, y1, x2, y2)

            candidates.append({
                "class_name": class_name,
                "conf": conf,
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
                "cx": cx,
                "cy": cy,
                "area": area
            })

        return candidates, current_target

    # =========================================================
    # TCP
    # =========================================================
    def ensure_connection(self):
        with self.sock_lock:
            if self.connected and self.sock is not None:
                return True

            try:
                if self.sock is not None:
                    try:
                        self.sock.close()
                    except Exception:
                        pass

                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(3.0)
                self.sock.connect((self.server_ip, self.server_port))
                self.connected = True
                rospy.loginfo("已连接到机器人 bridge: %s:%d", self.server_ip, self.server_port)
                self.append_log("Connected to robot bridge: {}:{}".format(self.server_ip, self.server_port))
                return True

            except Exception as e:
                self.connected = False
                rospy.logwarn_throttle(2.0, "连接机器人 bridge 失败: %s", e)
                return False

    def send_dict(self, data_dict):
        if not self.ensure_connection():
            return False

        msg = json.dumps(data_dict, ensure_ascii=False) + "\n"

        with self.sock_lock:
            try:
                self.sock.sendall(msg.encode("utf-8"))
                return True
            except Exception as e:
                rospy.logwarn_throttle(1.0, "发送检测结果失败: %s", e)
                self.append_log("Send failed: {}".format(e))
                self.connected = False
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None
                return False

    # =========================================================
    # 可视化
    # =========================================================
    def draw_visualization(self, image, selected_target, all_candidates, current_target):
        vis = image.copy()
        h, w = vis.shape[:2]

        cv2.line(vis, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (0, 255, 255), 2)
        cv2.line(vis, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (0, 255, 255), 2)

        for det in all_candidates:
            cv2.rectangle(vis, (det["x1"], det["y1"]), (det["x2"], det["y2"]), (180, 180, 180), 1)
            label = "{} {:.2f}".format(det["class_name"], det["conf"])
            cv2.putText(vis, label, (det["x1"], max(20, det["y1"] - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        if selected_target is not None:
            x1, y1, x2, y2 = selected_target["x1"], selected_target["y1"], selected_target["x2"], selected_target["y2"]
            cx, cy = selected_target["cx"], selected_target["cy"]
            conf = selected_target["conf"]
            cls = selected_target["class_name"]

            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(vis, (cx, cy), 5, (255, 0, 0), -1)

            label = "[SEND] {} {:.2f}".format(cls, conf)
            cv2.putText(vis, label, (x1, max(25, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        info1 = "Target Class: {}".format(current_target)
        info2 = "Server: {}:{}".format(self.server_ip, self.server_port)
        info3 = "Connected: {}".format(self.connected)

        cv2.putText(vis, info1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(vis, info2, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(vis, info3, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        return vis

    # =========================================================
    # 主循环
    # =========================================================
    def spin(self):
        rate = rospy.Rate(15)

        rospy.loginfo("YOLO Detector Sender UI Node 启动完成")
        rospy.loginfo("订阅 RGB: %s", self.rgb_topic)
        rospy.loginfo("TCP 发送到: %s:%d", self.server_ip, self.server_port)

        while not rospy.is_shutdown():
            self.update_ui()

            frame = None
            with self.img_lock:
                if self.latest_bgr is not None:
                    frame = self.latest_bgr.copy()

            if frame is None:
                rate.sleep()
                continue

            try:
                candidates, current_target = self.run_inference(frame)
                selected = self.select_target(candidates)

                if selected is not None:
                    self.last_target_center = (selected["cx"], selected["cy"])
                    self.last_detected = True

                    data_dict = {
                        "detected": True,
                        "class_name": selected["class_name"],
                        "confidence": float(selected["conf"]),
                        "x1": int(selected["x1"]),
                        "y1": int(selected["y1"]),
                        "x2": int(selected["x2"]),
                        "y2": int(selected["y2"]),
                        "center_x": int(selected["cx"]),
                        "center_y": int(selected["cy"])
                    }

                    self.send_dict(data_dict)

                    rospy.loginfo_throttle(
                        1.0,
                        "[Detection->TCP] target=%s conf=%.2f bbox=[%d,%d,%d,%d] center=(%d,%d)",
                        selected["class_name"], selected["conf"],
                        selected["x1"], selected["y1"], selected["x2"], selected["y2"],
                        selected["cx"], selected["cy"]
                    )
                else:
                    self.last_detected = False

                    if self.send_no_detection:
                        data_dict = {
                            "detected": False,
                            "class_name": current_target,
                            "confidence": 0.0,
                            "x1": 0,
                            "y1": 0,
                            "x2": 0,
                            "y2": 0,
                            "center_x": 0,
                            "center_y": 0
                        }
                        self.send_dict(data_dict)

                    rospy.loginfo_throttle(1.0, "[Detection->TCP] 未检测到目标类别: %s", current_target)

                if self.show_window:
                    vis = self.draw_visualization(frame, selected, candidates, current_target)
                    cv2.imshow("YOLO Detection Sender UI", vis)
                    cv2.waitKey(1)

            except Exception as e:
                rospy.logerr_throttle(1.0, "YOLO Sender UI 运行异常: %s", e)
                traceback.print_exc()

            rate.sleep()

        if self.show_window:
            cv2.destroyAllWindows()

    def on_shutdown(self):
        with self.sock_lock:
            try:
                if self.sock is not None:
                    self.sock.close()
            except Exception:
                pass


if __name__ == "__main__":
    try:
        node = YoloDetectorSenderUINode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
