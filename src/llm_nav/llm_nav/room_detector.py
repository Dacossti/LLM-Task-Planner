#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import json
import numpy as np
from collections import defaultdict
from importlib.resources import files

ROOM_FILE_PATH = files('llm_nav.data').joinpath('room_data.json')

class RoomDetector(Node):
    def __init__(self):
        super().__init__('room_detector')

        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        self.current_pose = None
        self.detected_objects = set()
        self.room_centers = defaultdict(list)

        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        self.timer = self.create_timer(2.0, self.classify_room)

        self.get_logger().info("Room detector initialized.")

    def pose_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_pose = (pos.x, pos.y)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        results = self.model.predict(cv_image, imgsz=640, conf=0.4)
        objects = set()
        for r in results:
            for cls in r.boxes.cls:
                label = self.model.names[int(cls)]
                objects.add(label.lower())
        self.detected_objects = objects

    def classify_room(self):
        if not self.current_pose or not self.detected_objects:
            return

        room = self.infer_room(self.detected_objects)
        if room:
            self.room_centers[room].append(self.current_pose)
            self.save_rooms()

    def infer_room(self, objects):
        if {'microwave', 'oven', 'fridge'}.intersection(objects):
            return 'kitchen'
        if {'bed', 'pillow'}.intersection(objects):
            return 'bedroom'
        if {'sofa', 'tv', 'bookshelf'}.intersection(objects):
            return 'living room'
        if {'toilet', 'sink'}.intersection(objects):
            return 'bathroom'
        if {'chair', 'desk', 'computer'}.intersection(objects):
            return 'office'
        return None

    def save_rooms(self):
        data = {}
        for room, poses in self.room_centers.items():
            avg_x = float(np.mean([p[0] for p in poses]))
            avg_y = float(np.mean([p[1] for p in poses]))
            data[room] = [avg_x, avg_y]  # ✅ Save as list [x, y]

        try:
            with open(ROOM_FILE_PATH, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f"Saved {len(data)} rooms to {ROOM_FILE_PATH}")
        except Exception as e:
            self.get_logger().error(f"Failed to save room data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RoomDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
