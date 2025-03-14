#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        self.path_subscription = self.create_subscription(
            Path,
            '/drone_autonav/local_plan',
            self.path_callback,
            10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/simple_drone/odom',
            self.odom_callback,
            10)
        self.cmd_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.current_path = None
        self.current_pose = None
        self.lookahead_distance = 0.5  # meters
        self.k_linear = 0.5
        self.k_angular = 1.0
        self.get_logger().info("Trajectory Controller node started.")

    def path_callback(self, msg):
        self.current_path = msg
        self.get_logger().info("Received new path with %d poses." % len(msg.poses))

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.control_loop()

    def control_loop(self):
        if self.current_path is None or self.current_pose is None or len(self.current_path.poses) == 0:
            return

        # Find the lookahead point on the path
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        lookahead_point = None
        for pose_stamped in self.current_path.poses:
            dx = pose_stamped.pose.position.x - current_x
            dy = pose_stamped.pose.position.y - current_y
            distance = math.hypot(dx, dy)
            if distance >= self.lookahead_distance:
                lookahead_point = pose_stamped.pose.position
                break

        if lookahead_point is None:
            lookahead_point = self.current_path.poses[-1].pose.position

        # Compute control errors
        error_x = lookahead_point.x - current_x
        error_y = lookahead_point.y - current_y
        desired_yaw = math.atan2(error_y, error_x)
        
        # Convert current orientation quaternion to yaw (for a planar case)
        q = self.current_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z)
        cosy_cosp = 1.0 - 2.0 * (q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        heading_error = self.normalize_angle(desired_yaw - current_yaw)
        distance_error = math.hypot(error_x, error_y)

        # Compute velocity commands (proportional controller)
        cmd = Twist()
        cmd.linear.x = self.k_linear * distance_error
        cmd.angular.z = self.k_angular * heading_error

        # Limit velocities
        max_linear = 1.0
        max_angular = 1.0
        cmd.linear.x = max(-max_linear, min(cmd.linear.x, max_linear))
        cmd.angular.z = max(-max_angular, min(cmd.angular.z, max_angular))

        self.cmd_pub.publish(cmd)
        self.get_logger().info("Published cmd_vel: linear %.2f angular %.2f" % (cmd.linear.x, cmd.angular.z))

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
