import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class MotionPlanner(Node):
    def __init__(self):  # Fixed constructor name
        super().__init__('motion_planner')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        self.current_pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.target_pose = [3.0, 4.0, math.pi/2]  # Default target
        self.control_timer = self.create_timer(0.1, self.update_control)

        # Control parameters
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.lin_step = 0.05
        self.ang_step = 0.1
        self.STOP_DISTANCE = 0.1  # meters
        self.ANGLE_TOLERANCE = 0.1  # radians

        self.get_logger().info("Motion Planner initialized!")

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.current_pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        ]

    def goal_cb(self, msg):
        # Proper quaternion to yaw conversion
        q = msg.pose.orientation
        self.target_pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        ]
        self.get_logger().info(f"New goal received: {self.target_pose}")

    def update_control(self):
        if not hasattr(self, 'current_pose'):
            self.get_logger().warn("Waiting for initial pose...")
            return

        # Calculate position error
        dx = self.target_pose[0] - self.current_pose[0]
        dy = self.target_pose[1] - self.current_pose[1]
        distance = math.hypot(dx, dy)

        # Calculate angular errors
        target_heading = math.atan2(dy, dx) if distance > 0.01 else self.target_pose[2]
        heading_error = self.normalize_angle(target_heading - self.current_pose[2])
        final_orientation_error = self.normalize_angle(self.target_pose[2] - self.current_pose[2])

        # Control logic
        if distance > self.STOP_DISTANCE:
            # Phase 1: Rotate to face target
            if abs(heading_error) > self.ANGLE_TOLERANCE:
                self.ang_vel = self.ang_step * heading_error
                self.lin_vel = 0.0
            # Phase 2: Move forward
            else:
                self.ang_vel = 0.0
                self.lin_vel = min(self.lin_vel + self.lin_step, 0.4)  # Max TurtleBot3 speed
        else:
            # Phase 3: Final orientation adjustment
            if abs(final_orientation_error) > self.ANGLE_TOLERANCE:
                self.ang_vel = self.ang_step * final_orientation_error
                self.lin_vel = 0.0
            else:
                self.ang_vel = 0.0
                self.lin_vel = 0.0

        # Publish command
        twist = Twist()
        twist.linear.x = self.lin_vel
        twist.angular.z = self.ang_vel
        self.cmd_vel_pub.publish(twist)
        self.get_logger().debug(f"Cmd_vel: lin={twist.linear.x:.2f}, ang={twist.angular.z:.2f}")

    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    planner = MotionPlanner()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info("Shutting down...")
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
