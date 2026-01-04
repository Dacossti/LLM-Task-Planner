import time
import math
import numpy as np
import yaml
import os
from geometry_msgs.msg import Twist
from llm_nav.thetastar import ThetaStar

class MotionController:
    def __init__(self, cmd_pub, occ_grid):
        self.cmd_pub = cmd_pub
        self.occ_grid = occ_grid
        self.map_resolution, self.map_origin = self._load_map_metadata()
        self.theta_star = ThetaStar(occ_grid)
        self.current_pose = (0.0, 0.0, 0.0)  # Initial pose (x, y, yaw)
        self._log_info(f"[MotionController] Initialized with map resolution: {self.map_resolution}")
        self.critical_distance = 0.35

    def _load_map_metadata(self):
        maps_dir = "/home/dacossti/llm_ros2_humble/src/llm_nav/maps"
        target_file = "my_map.yaml"
        yaml_path = os.path.join(maps_dir, target_file)
        if os.path.exists(yaml_path):
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                resolution = data['resolution']
                origin = tuple(data['origin'])
                print(f"[MotionController] Loaded map metadata from {target_file}")
                return resolution, origin
        else:
            print(f"[MotionController] Warning: {target_file} not found. Using defaults.")
            return 1.0, (0.0, 0.0, 0.0)

    def _log_info(self, msg):
        print(msg)  # You can replace this with ROS's logging system if necessary

    def _log_warn(self, msg):
        print(f"WARNING: {msg}")

    def grid_to_world(self, x, y):
        wx = x * self.map_resolution + self.map_origin[0]
        wy = y * self.map_resolution + self.map_origin[1]
        return (wx, wy)

    def is_free(self, x, y):
        if self.occ_grid is None:
            return True
        ix = int(round((x - self.map_origin[0]) / self.map_resolution))
        iy = int(round((y - self.map_origin[1]) / self.map_resolution))
        if 0 <= iy < self.occ_grid.shape[0] and 0 <= ix < self.occ_grid.shape[1]:
            return self.occ_grid[iy, ix] == 0
        return False

    def update_pose(self, delta_x, delta_y, delta_yaw):
        """Update the robot's pose."""
        x, y, yaw = self.current_pose
        new_x = x + delta_x * math.cos(yaw) - delta_y * math.sin(yaw)
        new_y = y + delta_x * math.sin(yaw) + delta_y * math.cos(yaw)
        new_yaw = (yaw + delta_yaw) % (2 * math.pi)  # Normalize yaw between 0 and 2*pi
        self.current_pose = (new_x, new_y, new_yaw)

    def follow_path(self, path, lidar_data=None):
        world_path = [self.grid_to_world(x, y) for (x, y) in path]
        i = 1

        while i < len(world_path):
            curr = world_path[i - 1]
            next_pt = world_path[i]

            if not self.safe_to_move(curr, next_pt, lidar_data):
                print(f"[Controller] Obstacle detected between {curr} and {next_pt}. Replanning...")
                goal_grid = path[-1]
                new_path = self.theta_star.plan(self.world_to_grid(curr), goal_grid,
                                                lidar_data=lidar_data)
                if new_path:
                    world_path = [self.grid_to_world(x, y) for (x, y) in new_path]
                    i = 1
                    continue
                else:
                    print("[Controller] Recovery: facing obstacle")
                    self.recover(lidar_data)  # Try to recover if obstacle detected
                    continue

            angle = math.atan2(next_pt[1] - curr[1], next_pt[0] - curr[0])
            distance = math.hypot(next_pt[0] - curr[0], next_pt[1] - curr[1])

            self.rotate(angle)
            self.move_straight(distance, lidar_data)
            i += 1

    def world_to_grid(self, pos):
        x, y = pos
        gx = int(round((x - self.map_origin[0]) / self.map_resolution))
        gy = int(round((y - self.map_origin[1]) / self.map_resolution))
        return (gx, gy)

    def rotate(self, angle):
        twist = Twist()
        twist.angular.z = 0.5 if angle > 0 else -0.5
        duration = abs(angle) / abs(twist.angular.z)
        start = time.time()
        while time.time() - start < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        self.stop()

        # Update pose after rotation
        _, _, yaw = self.current_pose
        self.update_pose(0, 0, angle)  # Update yaw

    def move_straight(self, distance, lidar_data=None):
        twist = Twist()
        twist.linear.x = 0.2
        steps = int(distance / (twist.linear.x * 0.1))
        for _ in range(steps):
            if lidar_data and self._is_facing_obstacle(lidar_data):
                print("[Controller] Obstacle ahead! Stopping.")
                self.stop()
                return
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

            # Update pose after moving
            delta_x = twist.linear.x * math.cos(self.current_pose[2])
            delta_y = twist.linear.x * math.sin(self.current_pose[2])
            self.update_pose(delta_x, delta_y, 0)

        self.stop()

    def stop(self):
        self.cmd_pub.publish(Twist())

    def safe_to_move(self, p1, p2, lidar_data=None):
        """Check if it's safe to move from point p1 to point p2, considering obstacles."""
        x1, y1 = p1
        x2, y2 = p2
        num_points = int(math.hypot(x2 - x1, y2 - y1) * 10)
        
        # Check the path between p1 and p2
        for i in range(num_points):
            t = i / num_points
            x = x1 * (1 - t) + x2 * t
            y = y1 * (1 - t) + y2 * t
            if not self.is_free(x, y):
                return False  # If there's an obstacle, stop

        # If lidar data is provided, check if the robot is facing an obstacle
        if lidar_data:
            min_distance = self._is_facing_obstacle(lidar_data)
            if min_distance is not None:
                return False  # If an obstacle is detected in front, stop moving

        return True  # It's safe to move
    

    def _is_facing_obstacle(self, lidar_data):
        """Check if there's an obstacle directly in front of the robot and return distance if any."""
        front_angle_min = -math.pi / 6  # -30 degrees
        front_angle_max = math.pi / 6   # 30 degrees
        min_distance = float('inf')

        # Iterate through lidar ranges
        for i, range_data in enumerate(lidar_data.ranges):
            if lidar_data.range_min < range_data < lidar_data.range_max:
                angle = lidar_data.angle_min + i * lidar_data.angle_increment
                # Only consider obstacles in the front
                if front_angle_min <= angle <= front_angle_max:
                    # Track the minimum distance in the front-facing range
                    if range_data < min_distance:
                        min_distance = range_data

        # If the closest obstacle in front is within the critical distance
        if min_distance < self.critical_distance:
            print(f"[Controller] Obstacle detected at {min_distance} meters in front.")
            return min_distance  # Return the distance to the closest obstacle

        return None  # No obstacle detected within the critical distance

    
    def follow_path(self, path, lidar_data=None):
        world_path = [self.grid_to_world(x, y) for (x, y) in path]
        i = 1

        while i < len(world_path):
            curr = world_path[i - 1]
            next_pt = world_path[i]

            # Check if the robot is facing an obstacle
            min_distance = self._is_facing_obstacle(lidar_data)
            if min_distance is not None:
                print(f"[Controller] Obstacle detected at {min_distance} meters in front. Stopping and avoiding.")
                self.stop()
                
                # Back up a bit to give space to rotate
                self.back_up()

                # Rotate to find a new path around the obstacle
                self.avoid_obstacle(curr, next_pt)

                # Restart from the current point
                continue

            # If it's safe, move towards the next point
            angle = math.atan2(next_pt[1] - curr[1], next_pt[0] - curr[0])
            distance = math.hypot(next_pt[0] - curr[0], next_pt[1] - curr[1])

            self.rotate(angle)
            self.move_straight(distance, lidar_data)
            i += 1

    def back_up(self, distance=0.2):
        """Backs up the robot to create space to rotate."""
        twist = Twist()
        twist.linear.x = -0.15  # Move backwards
        start = time.time()
        initial_position = self.current_pose  # Save current position before moving
        while time.time() - start < distance / abs(twist.linear.x):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        self.stop()

        # After backing up, update the current position
        # Here, we assume movement along the x-axis, adjust accordingly for more complex motion
        self.current_pose = (initial_position[0] - distance, initial_position[1])  # Update x-coordinate
        print(f"[Controller] Backed up. New position: {self.current_pose}")


    def avoid_obstacle(self, curr, next_pt):
        """Rotate to avoid obstacle and continue path."""
        print("[Controller] Rotating to find new path around obstacle.")
        
        # Rotate to find a clear path around the obstacle
        self.rotate(math.pi / 4)  # Rotate by 45 degrees to avoid obstacle

        # Update current pose after rotating
        robot_x, robot_y, robot_yaw = self.current_pose
        robot_yaw += math.pi / 4  # Update yaw after rotating (adjust this based on your coordinate system)
        self.current_pose = (robot_x, robot_y, robot_yaw)  # Update current position with the new yaw

        print(f"[Controller] Current position updated: {self.current_pose}")

        # Replan and continue moving towards the goal
        new_path = self.theta_star.plan(curr, next_pt)  # Re-plan
        if new_path:
            print("[Controller] New path found after avoiding obstacle.")
            self.follow_path(new_path)
        else:
            print("[Controller] No path found after obstacle avoidance.")



'''
    def recover(self, lidar_data):
        """Recovery behavior: Back up and rotate to avoid obstacle."""
        robot_pose = self.current_pose
        if robot_pose:
            x, y, yaw = robot_pose

            # Check if there's an obstacle directly in front
            obstacle_in_front = self._is_facing_obstacle(lidar_data)

            if obstacle_in_front:
                print("[Controller] Obstacle detected in front! Backing up and spinning to avoid.")

                # Backup for 1 second
                twist = Twist()
                twist.linear.x = -0.15
                self.cmd_pub.publish(twist)
                time.sleep(1.0)  # Backup for 1 second
                self.stop()

                # Spin 180 degrees to avoid the obstacle
                self.rotate(math.pi)  # 180 degree turn
                print("[Controller] Spinning 180 degrees.")

            # After spinning, resume normal behavior (optional: you can add a check to confirm that it's safe to move)
            print("[Controller] Moving again.")
        else:
            print("[Controller] Unable to recover, robot pose is not available.")
'''
