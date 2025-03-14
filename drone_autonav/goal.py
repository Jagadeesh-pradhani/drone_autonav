#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

class DroneGoalPublisher(Node):
    def __init__(self):
        super().__init__('drone_goal_publisher')
        # Publisher for goal points
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # Subscriber to know when a goal has been reached
        self.create_subscription(Empty, '/goal_reached', self.goal_reached_callback, 10)

        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.ugv_position = None
        # Define the five goals as (x, y, z)
        self.goals = [
            (-4.0, 4.0, 3.0),
            (-1.0, 4.0, 3.0),
            (-1.0, 0.0, 3.0),
            (-4.0, 0.0, 3.0),
            (-4.0, -4.0, 3.0),
            # (1.0, 0.0, 3.0)
        ]
        self.current_goal_index = 0

        # Publish the first goal immediately
        self.publish_goal()

    def odom_callback(self, msg : Odometry):
        """Callback for odometry messages."""

        self.ugv_position = msg.pose.pose.position
        self.ugv_orientation = msg.pose.pose.orientation
        # print(self.ugv_position)

    def publish_goal(self):
        """Publish the current goal to /drone_goal."""

        
        if self.current_goal_index < len(self.goals):
            goal = self.goals[self.current_goal_index]
            point_msg = PoseStamped()
            point_msg.pose.position.x, point_msg.pose.position.y, point_msg.pose.position.z = goal
            self.goal_publisher.publish(point_msg)
            self.get_logger().info(
                f'Published goal {self.current_goal_index+1}: (x: {point_msg.pose.position.x}, y: {point_msg.pose.position.y}, z: {point_msg.pose.position.z})'
            )
        else:
            point_msg = PoseStamped()
            point_msg.pose.position.x, point_msg.pose.position.y, point_msg.pose.position.z = self.ugv_position.x, self.ugv_position.y, 3.0
            self.goal_publisher.publish(point_msg)
            self.get_logger().info(
                f'Published goal {self.current_goal_index+1}: (x: {point_msg.pose.position.x}, y: {point_msg.pose.position.y}, z: {point_msg.pose.position.z})'
            )
            self.get_logger().info('Mission complete. All goals have been published.')
            exit()

    def goal_reached_callback(self, msg):
        """Callback when a goal is reached. Move to the next goal."""
        self.get_logger().info('Received goal reached signal.')
        self.current_goal_index += 1
        if self.current_goal_index < len(self.goals):
            self.publish_goal()
        else:
            self.get_logger().info('All goals reached. Mission completed.')
            self.publish_goal()

def main(args=None):
    rclpy.init(args=args)
    node = DroneGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
