#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from importlib.resources import files
import json
import os
import time
from llm_nav.nl_parser import llm_generate_plan
from visualization_msgs.msg import Marker

ROOM_FILE_PATH = files('llm_nav.data').joinpath('room_data.json')

class NavToPoseNode(Node):
    def __init__(self):
        super().__init__('nav_to_pose_node')

        self.declare_parameter('instruction', 'Go to the living room')
        self.instruction = self.get_parameter('instruction').get_parameter_value().string_value
        self.get_logger().info(f"User instruction: {self.instruction}")

        self.map_msg = None
        self.lidar_data = None
        self.current_pose = (0.0, 0.0, 0.0)  # x, y, yaw

        self.marker_pub = self.create_publisher(Marker, 'room_markers', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.detected_rooms = self.load_room_data()

        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.timer = self.create_timer(3.0, self.execute_llm_plan)

    def load_room_data(self):
        if not os.path.exists(ROOM_FILE_PATH):
            self.get_logger().warn(f"Rooms data file not found at {ROOM_FILE_PATH}")
            return {}
        try:
            with open(ROOM_FILE_PATH, 'r') as f:
                data = json.load(f)
                for room, pos in data.items():
                    self.publish_marker(room, pos[0], pos[1])
                return data
        except Exception as e:
            self.get_logger().error(f"Failed to load room data: {e}")
            return {}

    def publish_marker(self, room, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = room
        marker.id = abs(hash(room)) % 100000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.5
        marker.scale.z = 0.5
        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.text = room.upper()
        self.marker_pub.publish(marker)

    def publish_initial_pose(self, x, y, yaw=0.0):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.orientation.w = 1.0
        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[7] = 0.25
        pose_msg.pose.covariance[35] = 0.06853891909122467  # yaw variance
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(f"Published initial pose at ({x}, {y})")

    def map_callback(self, msg):
        self.map_msg = msg
        self.get_logger().info("Map received.")

    def lidar_callback(self, msg):
        self.lidar_data = msg

    def execute_llm_plan(self):
        if self.map_msg is None:
            self.get_logger().info("Waiting for map...")
            return

        if not self.detected_rooms:
            self.get_logger().info("Waiting for room data...")
            return

        if not self.navigator.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose action server not available.")
            return

        # Set initial pose before executing the plan
        self.publish_initial_pose(*self.current_pose)

        plan = llm_generate_plan(self.instruction)
        if not plan:
            self.get_logger().error("LLM failed to generate a plan.")
            return

        self.get_logger().info(f"LLM plan: {plan}")

        step = 1
        for action_type, target in plan:
            if action_type.lower() == 'navigate':
                room = target.lower().strip()
                goal = self.detected_rooms.get(room)
                if not goal:
                    self.get_logger().warn(f"Room '{room}' not found in room data.")
                    continue

                if self.send_nav_goal(goal[0], goal[1]):
                    self.get_logger().info(f"Step {step} to {room} completed.")
                    self.current_pose = (goal[0], goal[1], 0.0)
                    self.publish_initial_pose(*self.current_pose)
                    time.sleep(2)
                else:
                    self.get_logger().warn(f"Step {step} to {room} failed!")
                    break
                step += 1

        self.get_logger().info("All plan steps completed.")
        self.destroy_node()
        rclpy.shutdown()

    def send_nav_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending goal to ({x:.2f}, {y:.2f})")
        send_goal_future = self.navigator.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected or failed to be sent.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            self.get_logger().error("Goal execution failed to return a result.")
            return False

        if result.result.error_code == 0:
            self.get_logger().info("Goal succeeded.")
            return True
        else:
            self.get_logger().warn(f"Goal failed with error code: {result.result.error_code}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = NavToPoseNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
