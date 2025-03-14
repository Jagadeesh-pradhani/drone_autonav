#!/usr/bin/env python3
import math
import heapq
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, Twist, Point, Quaternion
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from std_msgs.msg import Empty

# Import the provided CollisionChecker and helper GridNode.
from drone_autonav.collision_checker import CollisionChecker
from drone_autonav.node import Node as GridNode  # Helper class representing a 2D grid node

class DroneAutonav(Node):
    def __init__(self):
        super().__init__('drone_autonav_node')

        # Declare parameters.
        self.declare_parameter("avoid_height", 2.5)
        self.declare_parameter("avoid_gap", 3)  # retained for compatibility

        # Subscribers for goal and clicked point (from RViz).
        self.subscription_goal = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10)
        self.subscription_clicked_point = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10)
        self.subscription_goal_reached = self.create_subscription(
            Empty, '/goal_reached', self.goal_reached_callback, 10)

        # Subscriber for current odometry.
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/simple_drone/odom',
            self.odom_callback,
            10)

        # Subscriber for map (OccupancyGrid).
        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            'projected_map',
            self.map_callback,
            10)

        # Publishers for velocity commands and path visualization.
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            '/simple_drone/cmd_vel',
            10)
        self.publisher_path = self.create_publisher(
            Path,
            'drone_path',
            10)

        # Drone state variables.
        self.current_position = None  # geometry_msgs/Point
        self.current_orientation = None  # geometry_msgs/Quaternion
        self.goal_position = None  # geometry_msgs/Point
        self.default_altitude = None  # default altitude from goal

        # Flag to indicate if the path has been modified to avoid collisions.
        self.path_modified = False
        self.new_goal = False

        # Planned path as a list of waypoints (tuples: (x, y, z)).
        self.path_waypoints = []

        # Control gains.
        self.kp_linear = 0.5
        self.kp_yaw = 0.5

        # For simple 2D grid planning, define planning bounds and grid resolution.
        self.planning_bounds = [-10.0, 10.0, -10.0, 10.0]  # [xmin, xmax, ymin, ymax]
        self.grid_resolution = 0.5

        # Collision checker instance (set when map is received).
        self.collision_checker = None

        # Timer for control loop (10 Hz).
        self.timer = self.create_timer(0.1, self.timer_callback)

    def goal_reached_callback(self, msg: Empty):
        self.get_logger().info('Received goal reached signal.')
        # When goal is reached, restore the default altitude.
        if self.path_modified:
            self.restore_default_altitude()
            # self.publish_path()
            self.path_modified = False

    def goal_callback(self, msg: PoseStamped):
        self.goal_position = msg.pose.position
        self.goal_position.z = 1.0
        self.default_altitude = self.goal_position.z
        self.new_goal = True
        self.get_logger().info(
            f"Received goal_pose: ({self.goal_position.x:.2f}, {self.goal_position.y:.2f}, {self.goal_position.z:.2f})"
        )
        if self.current_position is not None:
            self.plan_path()

    def clicked_point_callback(self, msg: PointStamped):
        self.goal_position = msg.point
        self.get_logger().info(
            f"Received clicked_point: ({self.goal_position.x:.2f}, {self.goal_position.y:.2f}, {self.goal_position.z:.2f})"
        )
        if self.current_position is not None:
            self.plan_path()

    def odom_callback(self, msg: Odometry):
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation

    def map_callback(self, msg: OccupancyGrid):
        if self.collision_checker is None:
            self.collision_checker = CollisionChecker(msg, robot_radius=0.3, planning_bounds=[20, 20])
            self.get_logger().info("Initialized CollisionChecker from map.")
        else:
            self.collision_checker.update_map(msg)
            self.get_logger().debug("CollisionChecker updated from map.")

    def world_to_grid(self, point: Point) -> tuple:
        xmin, xmax, ymin, ymax = self.planning_bounds
        i = int((point.x - xmin) / self.grid_resolution)
        j = int((point.y - ymin) / self.grid_resolution)
        return (i, j)

    def grid_to_world(self, idx: tuple) -> tuple:
        xmin, xmax, ymin, ymax = self.planning_bounds
        x = xmin + (idx[0] + 0.5) * self.grid_resolution
        y = ymin + (idx[1] + 0.5) * self.grid_resolution
        z = self.current_position.z if self.current_position else 0.0
        return (x, y, z)

    def heuristic(self, a: tuple, b: tuple) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def get_neighbors(self, current: tuple) -> list:
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (current[0] + dx, current[1] + dy)
                xmin, xmax, ymin, ymax = self.planning_bounds
                max_i = int((xmax - xmin) / self.grid_resolution)
                max_j = int((ymax - ymin) / self.grid_resolution)
                if 0 <= neighbor[0] < max_i and 0 <= neighbor[1] < max_j:
                    neighbors.append(neighbor)
        return neighbors

    def plan_path(self):
        if self.current_position is None or self.goal_position is None:
            self.get_logger().warn("Cannot plan path without current and goal positions.")
            return

        start = self.world_to_grid(self.current_position)
        goal = self.world_to_grid(self.goal_position)
        self.get_logger().info(f"Planning path from {start} to {goal}...")

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        cost_so_far = {start: 0}
        found = False

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                found = True
                break
            for neighbor in self.get_neighbors(current):
                new_cost = cost_so_far[current] + self.heuristic(current, neighbor)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current

        if not found:
            self.get_logger().error("Path planning failed: No valid path found!")
            self.path_waypoints = []
            return

        current = goal
        grid_path = [current]
        while current != start:
            current = came_from[current]
            grid_path.append(current)
        grid_path.reverse()

        # Convert grid indices to world coordinates.
        coarse_path = [self.grid_to_world(idx) for idx in grid_path]
        self.path_waypoints = self.densify_path(coarse_path, interp_resolution=0.1)
        self.restore_default_altitude()
        # self.path_waypoints.append((self.goal_position.x, self.goal_position.y, self.goal_position.z))
        self.get_logger().info(f"Path planned with {len(self.path_waypoints)} densified waypoints.")
        self.publish_path()
        # Reset modification flag since a new path is planned.
        self.path_modified = False

    def densify_path(self, waypoints: list, interp_resolution: float = 0.1) -> list:
        dense_path = []
        for i in range(len(waypoints) - 1):
            p_start = np.array(waypoints[i])
            p_end = np.array(waypoints[i+1])
            seg = p_end - p_start
            seg_length = np.linalg.norm(seg)
            if seg_length < 1e-6:
                continue
            num_points = max(2, int(seg_length / interp_resolution) + 1)
            for j in range(num_points):
                t = j / (num_points - 1)
                p_interp = (1 - t) * p_start + t * p_end
                dense_path.append(tuple(p_interp))
        return dense_path

    def publish_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for waypoint in self.path_waypoints:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.publisher_path.publish(path_msg)
        self.get_logger().info("Published path to 'drone_path'.")

    def modify_path_for_collision(self):
        """
        If the collision checker detects a collision along the path, modify the entire path altitude.
        Here, we set the altitude for all waypoints to: current altitude + avoid_height.
        This modification is done only once per collision event.
        """
        if not self.path_waypoints or not self.collision_checker:
            return

        avoid_offset = self.get_parameter("avoid_height").value
        new_altitude = avoid_offset

        self.path_waypoints = [(wp[0], wp[1], new_altitude) for wp in self.path_waypoints]
        self.path_waypoints[-1] = (self.goal_position.x, self.goal_position.y, self.goal_position.z)
        self.path_modified = True
        self.get_logger().info("Modified entire path altitude based on current position.")

    def restore_default_altitude(self):
        """
        Restore the planned path altitude to the default altitude (from the goal).
        This is called only once when the path becomes clear.
        """
        if not self.path_waypoints or self.default_altitude is None:
            return

        self.path_waypoints = [(wp[0], wp[1], self.default_altitude) for wp in self.path_waypoints]
        self.path_modified = False
        self.get_logger().info("Restored path altitude to default.")

    def timer_callback(self):
        if self.path_waypoints and self.current_position is not None and self.current_orientation is not None:
            if self.collision_checker and not self.path_modified:
                
                path_nodes = [GridNode(x=wp[0], y=wp[1]) for wp in self.path_waypoints]
                occ_nodes = self.collision_checker.is_path_free(path_nodes)
                # Modify path only once when collision is first detected.
                if occ_nodes and not self.path_modified and self.new_goal:
                    self.new_goal = False
                    self.get_logger().warn("Path is in collision. Increasing altitude from current position.")
                    self.modify_path_for_collision()
                    self.publish_path()
                # If path was modified and now collision is cleared, restore default altitude once.
                # elif self.path_modified and not occ_nodes:
                #     self.get_logger().info("Path is clear. Restoring default altitude.")
                #     self.restore_default_altitude()
                #     self.publish_path()

            next_wp = self.path_waypoints[0]
            curr = np.array([self.current_position.x, self.current_position.y, self.current_position.z])
            target = np.array(next_wp)
            error_vec = target - curr
            distance = np.linalg.norm(error_vec)

            desired_yaw = math.atan2(error_vec[1], error_vec[0])
            current_yaw = self.quaternion_to_yaw(self.current_orientation)
            yaw_error = math.atan2(math.sin(desired_yaw - current_yaw), math.cos(desired_yaw - current_yaw))

            raw_factor = max(0.0, math.cos(yaw_error))
            alignment_factor = 0.2 + 0.8 * raw_factor
            forward_speed = self.kp_linear * distance * alignment_factor

            twist = Twist()
            twist.linear.x = forward_speed * math.cos(desired_yaw)
            twist.linear.y = forward_speed * math.sin(desired_yaw)
            twist.linear.z = self.kp_linear * (next_wp[2] - self.current_position.z)
            twist.angular.z = self.kp_yaw * yaw_error

            # self.publisher_cmd_vel.publish(twist)
            self.get_logger().debug(
                f"Distance: {distance:.2f}, Yaw error: {yaw_error:.2f}, "
                f"Alignment factor: {alignment_factor:.2f}, Forward speed: {forward_speed:.2f}"
            )
            if distance < 0.2:
                self.get_logger().info("Reached waypoint, proceeding to next...")
                self.path_waypoints.pop(0)
                if not self.path_waypoints:
                    self.get_logger().info("Final goal reached!")
        else:
            if self.goal_position is not None and self.current_position is not None:
                self.get_logger().debug("No path available; re-planning...")
                self.plan_path()

    def quaternion_to_yaw(self, q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = DroneAutonav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
