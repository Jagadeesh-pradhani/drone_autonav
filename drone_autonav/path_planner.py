#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
import math
import heapq

class AStarPlanner:
    def __init__(self, grid, resolution, origin):
        self.grid = grid  # 2D numpy array (height x width)
        self.resolution = resolution
        self.origin = origin  # [x0, y0]
        self.height, self.width = grid.shape

    def heuristic(self, a, b):
        # Euclidean distance
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def get_neighbors(self, node):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                x2 = node[0] + dx
                y2 = node[1] + dy
                if 0 <= x2 < self.width and 0 <= y2 < self.height:
                    if self.grid[y2, x2] < 50:  # free cell threshold
                        neighbors.append((x2, y2))
        return neighbors

    def plan(self, start, goal):
        # start and goal are (x,y) indices in grid coordinates
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + self.heuristic(current, neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return None  # No path found

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/drone_autonav/occupancy_grid',
            self.map_callback,
            10)
        self.goal_subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.goal_callback,
            10)
        self.path_pub = self.create_publisher(Path, '/drone_autonav/local_plan', 10)
        self.latest_grid = None
        self.grid_info = None
        self.get_logger().info("Path Planner node started.")

    def map_callback(self, msg):
        # Convert OccupancyGrid data to 2D numpy array
        width = msg.info.width
        height = msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.latest_grid = grid
        self.grid_info = msg.info

    def goal_callback(self, msg):
        if self.latest_grid is None:
            self.get_logger().warn("No map received yet.")
            return

        # Convert goal point from world coordinates to grid indices
        x_goal = msg.point.x
        y_goal = msg.point.y
        x0 = self.grid_info.origin.position.x
        y0 = self.grid_info.origin.position.y
        ix_goal = int((x_goal - x0) / self.grid_info.resolution)
        iy_goal = int((y_goal - y0) / self.grid_info.resolution)
        goal_idx = (ix_goal, iy_goal)

        # For simplicity, assume the start is at the center of the grid.
        # In practice, use the drone's current position from odom.
        start_idx = (self.latest_grid.shape[1] // 2, self.latest_grid.shape[0] // 2)

        self.get_logger().info(f"Planning from {start_idx} to {goal_idx}")
        planner = AStarPlanner(self.latest_grid, self.grid_info.resolution, [x0, y0])
        path_indices = planner.plan(start_idx, goal_idx)

        if path_indices is None:
            self.get_logger().error("No path found!")
            return

        # Convert grid indices back to world coordinates and populate Path message
        path_msg = Path()
        path_msg.header.stamp = msg.header.stamp
        path_msg.header.frame_id = msg.header.frame_id
        for (ix, iy) in path_indices:
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = msg.header.frame_id
            pose.pose.position.x = x0 + (ix + 0.5) * self.grid_info.resolution
            pose.pose.position.y = y0 + (iy + 0.5) * self.grid_info.resolution
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        self.get_logger().info("Path planned and published.")

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
