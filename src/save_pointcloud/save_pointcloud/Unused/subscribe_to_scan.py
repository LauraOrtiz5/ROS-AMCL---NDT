import rclpy
from rclpy.node import Node

import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

import numpy as np
#import struct


class ScanToPointCloud2(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud2')
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/pointcloud', 10)

    def scan_callback(self,scan):
        cloud = PointCloud2()                           # Create empty pointcloud
        cloud.header.stamp = scan.header.stamp          # Fill headers as per scan topic
        cloud.header.frame_id = scan.header.frame_id

        # Convert Scan into array
        for i in range(len(scan.ranges)):
            if scan.ranges[i] > 0 and np.isfinite(scan.ranges[i]):
                x = scan.ranges[i] * np.cos(scan.angle_min + i * scan.angle_increment)
                y = scan.ranges[i] * np.sin(scan.angle_min + i * scan.angle_increment)
                points = np.array((x, y, 0))
                #points = np.array([(scan.ranges[i] * np.cos(scan.angle_min + i * scan.angle_increment), scan.ranges[i] * np.sin(scan.angle_min + i * scan.angle_increment), 0.0) for i in range(len(scan.ranges))], dtype=[('x', np.float32),('y', np.float32), ('z', np.float32)])
        
        # Fill the empty pointcloud
        cloud.width = len(points)
        cloud.height = 1
        cloud.fields = [sensor_msgs.PointField(name='x', offset=0, datatype=sensor_msgs.PointField.FLOAT32, count=1),
                        sensor_msgs.PointField(name='y', offset=4, datatype=sensor_msgs.PointField.FLOAT32, count=1),
                        sensor_msgs.PointField(name='z', offset=8, datatype=sensor_msgs.PointField.FLOAT32, count=1)]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = False
        #cloud.data = struct.pack("%df" % (len(points)*3), *points.flat)
        points_flat = np.column_stack((x, y, 0))
        points_flat = points_flat.transpose()
        cloud.data = points_flat.astype(np.float32).tobytes()

        self.pointcloud_pub.publish(cloud)              # Publish the pointcloud


def main(args=None):
    rclpy.init(args=args)
    scan_to_pointcloud2 = ScanToPointCloud2()
    rclpy.spin(scan_to_pointcloud2)
    scan_to_pointcloud2.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
