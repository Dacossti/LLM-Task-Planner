#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import json
import os
import time
from llm_nav.nl_parser import llm_generate_plan
from visualization_msgs.msg import Marker
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from llm_nav.thetastar import ThetaStar
from llm_nav.motion_controller import MotionController
import numpy as np
from geometry_msgs.msg import Twist
from importlib.resources import files


ROOM_FILE_PATH = files('llm_nav.data').joinpath('room_data_house.json')

class NavToPoseNode(Node):
    def __init__(self):
        super().__init__('nav_to_pose_node')

        self.declare_parameter('instruction', 'Go to the living room')
        self.declare_parameter('use_nav2', False)
        self.instruction = self.get_parameter('instruction').get_parameter_value().string_value
        self.plan = llm_generate_plan(self.instruction)
        self.use_nav2 = self.get_parameter('use_nav2').get_parameter_value().bool_value
        self.get_logger().info(f"User instruction: {self.instruction}")
        self.get_logger().info(f"Using Nav2 stack: {self.use_nav2}")

        self.map_msg = None
        self.lidar_data = None
        self.occ_grid_np = None
        self.current_pose = (0.0, 0.0, 0.0)

        self.marker_pub = self.create_publisher(Marker, 'room_markers', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.detected_rooms = self.load_room_data()

        if self.use_nav2:
            self.navigator = BasicNavigator()

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 100)

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
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(f"Published initial pose at ({x}, {y})")

    def map_callback(self, msg):
        self.map_msg = msg
        self.occ_grid_np = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.get_logger().info("Map received.")

    def lidar_callback(self, msg):
        self.lidar_data = msg

    def get_pose_stamped(self, x, y, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def execute_llm_plan(self):
        if self.map_msg is None or self.occ_grid_np is None:
            self.get_logger().info("Waiting for map...")
            return
        if not self.detected_rooms:
            self.get_logger().info("Waiting for room data...")
            return
        
        self.publish_initial_pose(*self.current_pose)
        time.sleep(5)

        if self.use_nav2:
            self.navigator.waitUntilNav2Active()
            self.navigator.setInitialPose(self.get_pose_stamped(*self.current_pose))

        if not self.plan:
            self.plan = llm_generate_plan(self.instruction)
            self.get_logger().error("Waiting for a valid plan...")
            return

        self.get_logger().info(f"LLM plan: {self.plan}")

        step = 1
        for action_type, target in self.plan:
            if action_type.lower() == 'navigate':
                room = target.lower().strip()
                goal = self.detected_rooms.get(room)
                if not goal:
                    self.get_logger().warn(f"Room '{room}' not found in room data.")
                    continue

                success = self.send_nav_goal(goal) if self.use_nav2 else self.follow_custom_path(goal)

                if success:
                    self.get_logger().info(f"Path {step} to location '{room}' completed!")
                    self.current_pose = (goal[0], goal[1], 0.0)
                    time.sleep(2)
                else:
                    self.get_logger().warn(f"Path {step} to location '{room}' failed!")
                    break
                step += 1

        self.get_logger().info("All plan steps are successfully completed.")
        if self.use_nav2:
            self.navigator.lifecycleShutdown()
        self.destroy_node()
        rclpy.shutdown()

    def send_nav_goal(self, goal):
        goal_pose = self.get_pose_stamped(goal[0], goal[1], 0.0)
        self.get_logger().info(f"Sending goal to ({goal[0]:.2f}, {goal[1]:.2f})")

        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            time.sleep(0.5)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded.")
            return True
        else:
            self.get_logger().warn(f"Goal failed with status: {result}")
            return False

    def follow_custom_path(self, goal):

        self.planner = ThetaStar(self.occ_grid_np)

        self.get_logger().info(f"Planning path from [{self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}] to {goal}")

        if not self.planner.is_free(*self.current_pose):
            self.get_logger().warn(f"Start {self.current_pose} is not free.")
        if not self.planner.is_free(*goal):
            self.get_logger().warn(f"Goal {goal} is not free.")

        # Pass lidar_data and the correct robot pose to the planner
        path = self.planner.plan(self.current_pose[:2], goal, lidar_data=self.lidar_data)

        if not path or len(path) < 2:
            self.get_logger().warn("No valid path found by Theta* planner.")
            return False

        self.controller = MotionController(self.cmd_pub, occ_grid=self.occ_grid_np)
        self.controller.follow_path(path, lidar_data=self.lidar_data)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = NavToPoseNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
