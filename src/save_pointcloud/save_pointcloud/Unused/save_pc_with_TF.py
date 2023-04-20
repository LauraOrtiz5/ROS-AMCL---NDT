import os
import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
import time
import pandas as pd

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

    # Save the point cloud data in a table called table_scan
    timestamp = time.time()
    data = {'timestamp': [timestamp]*len(point_cloud_data), 'x': [p[0] for p in point_cloud_data], 'y': [p[1] for p in point_cloud_data], 'z': [p[2] for p in point_cloud_data]}
    table_scan = pd.DataFrame(data)

    # Save the table in a CSV file
    table_scan.to_csv('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/pcd_files/Trial.csv', index=False)

def main(args=None):

    rclpy.init(args=args)
    node = rclpy.create_node('scan_to_pcd')
    qos_profile = QoSProfile(depth=10)
    sub = node.create_subscription(LaserScan, '/scan', scan_callback, qos_profile=qos_profile)
    rclpy.spin(node)

if __name__=='__main__':
    main()
