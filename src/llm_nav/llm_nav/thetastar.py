import heapq
import math
import os
import yaml
import numpy as np

class Node:
    def __init__(self, x, y, g=float('inf'), h=0.0, parent=None):
        self.x = int(x)
        self.y = int(y)
        self.g = g
        self.h = h
        self.parent = parent

    def __lt__(self, other):
        return (self.g + self.h) < (other.g + other.h)

def curved_line_of_sight(occ_grid, p1, p2, steps=20):
    """Check line of sight between two points by approximating a curved path."""
    x0, y0 = p1
    x1, y1 = p2
    # Interpolate between p1 and p2, creating a set of intermediate points
    for i in range(steps + 1):
        t = i / steps
        x = x0 + (x1 - x0) * t
        y = y0 + (y1 - y0) * t
        grid_x, grid_y = int(x), int(y)

        # Check if the interpolated point is inside bounds and not an obstacle
        if not (0 <= grid_x < occ_grid.shape[1] and 0 <= grid_y < occ_grid.shape[0]):
            return False
        if occ_grid[grid_y, grid_x] > 0:
            return False
    return True

def heuristic(p1, p2):
    """Heuristic function (Euclidean distance)."""
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def reconstruct_path(node):
    """Reconstruct the path from the start to the goal."""
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    path.reverse()
    return path

class ThetaStar:
    def __init__(self, occ_grid):
        self.occ_grid = occ_grid
        self.height, self.width = occ_grid.shape
        self.map_resolution, self.map_origin = self._load_map_metadata()

    def _load_map_metadata(self):
        maps_dir = "/home/dacossti/llm_ros2_humble/src/llm_nav/maps"
        yaml_path = os.path.join(maps_dir, "my_map.yaml")
        if os.path.exists(yaml_path):
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                return data['resolution'], data['origin']
        else:
            print("[Custom_Theta*] Failed to load map metadata, using defaults.")
            return 0.05, (0.0, 0.0, 0.0)

    def world_to_map(self, x, y):
        """Convert world coordinates to map indices."""
        mx = int((x - self.map_origin[0]) / self.map_resolution)
        my = int((y - self.map_origin[1]) / self.map_resolution)
        return mx, my

    def is_free(self, x, y, lidar_data=None):
        """Check if a cell in the grid is free of obstacles, considering lidar data."""
        grid_x = int(round((x - self.map_origin[0]) / self.map_resolution))
        grid_y = int(round((y - self.map_origin[1]) / self.map_resolution))
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            # Check if cell is free from map grid
            if self.occ_grid[grid_y, grid_x] == 0:
                # If lidar data is provided, check if there are obstacles detected by lidar
                if lidar_data:
                    return not self._check_lidar_obstacle(grid_x, grid_y, lidar_data)
                return True
        return False

    def _check_lidar_obstacle(self, grid_x, grid_y, lidar_msg):
        """Check for obstacles detected by lidar within a specific grid cell."""
        for i in range(len(lidar_msg.ranges)):
            angle = lidar_msg.angle_min + i * lidar_msg.angle_increment
            range_val = lidar_msg.ranges[i]
            if lidar_msg.range_min < range_val < lidar_msg.range_max:
                # Calculate lidar point in world coordinates
                obs_x = lidar_msg.range_min * math.cos(angle)
                obs_y = lidar_msg.range_min * math.sin(angle)
                obs_grid_x, obs_grid_y = self.world_to_map(obs_x, obs_y)

                # Check if the lidar point is in the vicinity of the grid cell
                if abs(obs_grid_x - grid_x) < 1 and abs(obs_grid_y - grid_y) < 1:
                    return True
        return False

    def plan(self, start_world, goal_world, lidar_data=None, max_iterations=5000):
        """Plan a path from start to goal using A* algorithm, incorporating lidar data."""
        print(f"[Custom_Theta*] Planning from {start_world} to {goal_world}...")

        start = self.world_to_map(*start_world)
        goal = self.world_to_map(*goal_world)

        if not self.is_free(*start_world, lidar_data=lidar_data):
            print(f"[Custom_Theta*] Start position {start} is not free.")
            return []

        if not self.is_free(*goal_world, lidar_data=lidar_data):
            print(f"[Custom_Theta*] Goal position {goal} is not free.")
            return []

        start_node = Node(*start, g=0.0, h=heuristic(start, goal))
        goal_node = Node(*goal)
        open_set = [start_node]
        closed_set = set()
        nodes = {(start_node.x, start_node.y): start_node}

        for iterations in range(max_iterations):
            if not open_set:
                break
            current = heapq.heappop(open_set)

            if (current.x, current.y) == (goal_node.x, goal_node.y):
                print(f"[Custom_Theta*] Goal reached in {iterations} iterations.")
                return reconstruct_path(current)

            closed_set.add((current.x, current.y))

            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue

                    nx, ny = current.x + dx, current.y + dy
                    wx = nx * self.map_resolution + self.map_origin[0]
                    wy = ny * self.map_resolution + self.map_origin[1]

                    if not self.is_free(wx, wy, lidar_data=lidar_data):
                        continue
                    if (nx, ny) in closed_set:
                        continue

                    neighbor = nodes.get((nx, ny))
                    if not neighbor:
                        neighbor = Node(nx, ny)
                        nodes[(nx, ny)] = neighbor

                    # Check line of sight for obstacles
                    if current.parent and curved_line_of_sight(self.occ_grid, (current.parent.x, current.parent.y), (nx, ny)):
                        new_g = current.parent.g + heuristic((current.parent.x, current.parent.y), (nx, ny))
                        parent = current.parent
                    else:
                        new_g = current.g + heuristic((current.x, current.y), (nx, ny))
                        parent = current

                    if new_g < neighbor.g:
                        neighbor.g = new_g
                        neighbor.h = heuristic((nx, ny), (goal_node.x, goal_node.y))
                        neighbor.parent = parent
                        heapq.heappush(open_set, neighbor)

        print(f"[Custom_Theta*] No path found after {max_iterations} iterations.")
        return []
