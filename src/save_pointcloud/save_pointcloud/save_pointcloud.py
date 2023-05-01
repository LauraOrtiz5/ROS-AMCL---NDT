import os
import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile

# Create an empty pointcloud
point_cloud_data = []

def scan_callback(scan):
    ranges = np.array(scan.ranges)
    angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

    # Convert any posible NaN or inf values into 0
    ranges[np.isnan(ranges)]=0
    angles[np.isnan(angles)]=0
    ranges[np.isinf(ranges)]=0
    angles[np.isinf(angles)]=0

    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    z = np.zeros(len(ranges))
    points = np.vstack((x,y,z)).T

    # Append the points to list
    point_cloud_data.extend(points.tolist())

    with open('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/pcd_files/Trial.pcd','wb') as f:
        f.write(b'# .PCD v0.7 - Point Cloud Data file format\n')
        f.write(b'VERSION 0.7\n')
        f.write(b'FIELDS x y z\n')
        f.write(b'SIZE 4 4 4\n')
        f.write(b'TYPE F F F\n')
        f.write(b'COUNT 1 1 1\n')
        f.write(b'WIDTH %d\n' % len(point_cloud_data))
        f.write(b'HEIGHT 1\n')
        f.write(b'VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(b'POINTS %d\n' % len(point_cloud_data))
        f.write(b'DATA binary\n')
        np.array(point_cloud_data, dtype='float32').tofile(f)


def main(args=None):

    rclpy.init(args=args)
    node = rclpy.create_node('scan_to_pcd') #Not being created ???
    qos_profile = QoSProfile(depth=10)
    sub = node.create_subscription(LaserScan, '/lidar', scan_callback, qos_profile=qos_profile)
    rclpy.spin(node)

if __name__=='__main__':
    main()
