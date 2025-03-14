#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import cv2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

# Camera intrinsics (assumed values, adjust as needed)
FX = 320.0  # focal length in pixels
FY = 320.0
CX = 320.0  # principal point x
CY = 240.0

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/simple_drone/front/depth/image_raw',
            self.depth_callback,
            10)
        self.pc_pub = self.create_publisher(PointCloud2, '/drone_autonav/pointcloud', 10)
        self.get_logger().info("Depth Processor node started.")

    def depth_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image (assuming 32FC1 depth image)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error("Failed to convert depth image: %s" % str(e))
            return

        # Get image dimensions
        height, width = depth_image.shape

        # Create list for point cloud points
        points = []
        for v in range(0, height, 4):  # downsample for speed
            for u in range(0, width, 4):
                z = depth_image[v, u]
                # Ignore invalid measurements
                if np.isnan(z) or z <= 0.1 or z > 10.0:
                    continue
                # Reconstruct 3D point in camera frame
                x = (u - CX) * z / FX
                y = (v - CY) * z / FY
                points.append([x, y, z])
        
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        # Create PointCloud2 message
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        pc2_msg = pc2.create_cloud(header, fields, points)
        self.pc_pub.publish(pc2_msg)
        self.get_logger().info("Published pointcloud with %d points." % len(points))

def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
