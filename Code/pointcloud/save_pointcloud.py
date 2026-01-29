#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import struct
import sys

class PointCloudSaver(Node):
    def __init__(self, topic_name, output_file):
        super().__init__('pointcloud_saver')
        self.output_file = output_file
        self.subscription = self.create_subscription(
            PointCloud2,
            topic_name,
            self.pointcloud_callback,
            10
        )
        self.get_logger().info(f'Listening to topic: {topic_name}')
        self.get_logger().info(f'Will save to: {output_file}')
        self.get_logger().info('Waiting for point cloud message...')

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received point cloud, processing...')

        # Extract points from PointCloud2 message
        points = []
        colors = []
        has_rgb = False

        # Check if RGB data is available
        field_names = [field.name for field in msg.fields]
        has_rgb = 'rgb' in field_names or 'rgba' in field_names

        # Read point cloud data
        for point in pc2.read_points(msg, skip_nans=True):
            x, y, z = point[0], point[1], point[2]
            points.append([x, y, z])

            if has_rgb and len(point) > 3:
                # Extract RGB from the packed float
                rgb = point[3]
                if isinstance(rgb, float):
                    rgb_int = struct.unpack('I', struct.pack('f', rgb))[0]
                    r = (rgb_int >> 16) & 0xFF
                    g = (rgb_int >> 8) & 0xFF
                    b = rgb_int & 0xFF
                    colors.append([r, g, b])

        points = np.array(points)

        self.get_logger().info(f'Extracted {len(points)} points')

        # Save to PLY file
        self.save_ply(points, colors if has_rgb else None)

        self.get_logger().info(f'Successfully saved to {self.output_file}')
        self.get_logger().info('Shutting down...')

        # Shutdown after saving
        rclpy.shutdown()

    def save_ply(self, points, colors=None):
        """Save points to PLY format"""
        with open(self.output_file, 'w') as f:
            # Write header
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write(f'element vertex {len(points)}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')

            if colors is not None and len(colors) > 0:
                f.write('property uchar red\n')
                f.write('property uchar green\n')
                f.write('property uchar blue\n')

            f.write('end_header\n')

            # Write data
            for i, point in enumerate(points):
                if colors is not None and i < len(colors):
                    f.write(f'{point[0]} {point[1]} {point[2]} {colors[i][0]} {colors[i][1]} {colors[i][2]}\n')
                else:
                    f.write(f'{point[0]} {point[1]} {point[2]}\n')

def main(args=None):
    if len(sys.argv) < 2:
        print('Usage: python3 save_pointcloud.py <output_file.ply> [topic_name]')
        print('Example: python3 save_pointcloud.py my_cloud.ply /rgbd_camera/points')
        sys.exit(1)

    output_file = sys.argv[1]
    topic_name = sys.argv[2] if len(sys.argv) > 2 else '/rgbd_camera/points'

    if not output_file.endswith('.ply'):
        output_file += '.ply'

    rclpy.init(args=args)

    saver = PointCloudSaver(topic_name, output_file)

    try:
        rclpy.spin(saver)
    except KeyboardInterrupt:
        pass
    finally:
        saver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
