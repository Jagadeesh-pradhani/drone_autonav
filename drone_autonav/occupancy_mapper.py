#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class OccupancyMapper(Node):
    def __init__(self):
        super().__init__('occupancy_mapper')
        # Subscribe to the point cloud data from the front camera
        self.pc_subscription = self.create_subscription(
            PointCloud2,
            '/simple_drone/front/points',
            self.pointcloud_callback,
            10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/drone_autonav/occupancy_grid', 10)
        # Define grid parameters
        self.grid_resolution = 0.2  # meters per cell
        self.grid_size = 100  # grid will be grid_size x grid_size cells
        # Origin such that the grid is centered around (0,0) in world frame
        self.origin = [-self.grid_size/2 * self.grid_resolution,
                       -self.grid_size/2 * self.grid_resolution]
        self.get_logger().info("Occupancy Mapper node started, subscribing to /simple_drone/front/points.")

    def pointcloud_callback(self, msg):
        # Create an empty occupancy grid (0: free, 100: occupied)
        grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)

        # Process each point in the point cloud
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            # Only consider points with a reasonable height (for a mostly planar map)
            if z < 0.2 or z > 2.0:
                continue
            # Convert world x,y into grid indices
            ix = int((x - self.origin[0]) / self.grid_resolution)
            iy = int((y - self.origin[1]) / self.grid_resolution)
            if 0 <= ix < self.grid_size and 0 <= iy < self.grid_size:
                grid[iy, ix] = 100  # mark as occupied

        # Create and populate OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = msg.header.stamp
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.info.resolution = self.grid_resolution
        occupancy_grid.info.width = self.grid_size
        occupancy_grid.info.height = self.grid_size
        occupancy_grid.info.origin.position.x = self.origin[0]
        occupancy_grid.info.origin.position.y = self.origin[1]
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0

        # Flatten grid row-major
        occupancy_grid.data = grid.flatten().tolist()
        self.map_pub.publish(occupancy_grid)
        self.get_logger().info("Published occupancy grid.")

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
