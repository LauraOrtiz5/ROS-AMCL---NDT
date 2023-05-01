import rclpy
from rclpy.node import Node

import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

import numpy as np
#import struct


class MapToPointCloud2(Node):
    def __init__(self):
        super().__init__('map_to_pointcloud2')
        self.map_sub = self.create_subscription(LaserScan, '/map', self.map_callback, 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/map_pc', 10)

    def map_callback(self,map):
        cloud = PointCloud2()                           # Create empty pointcloud
        cloud.header.stamp = map.header.stamp          # Fill headers as per map topic
        cloud.header.frame_id = map.header.frame_id

        # Convert map into array
        for i in range(len(map.ranges)):
            if map.ranges[i] > 0 and np.isfinite(map.ranges[i]):
                x = map.ranges[i] * np.cos(map.angle_min + i * map.angle_increment)
                y = map.ranges[i] * np.sin(map.angle_min + i * map.angle_increment)
                points = np.array((x, y, 0))
                #points = np.array([(map.ranges[i] * np.cos(map.angle_min + i * map.angle_increment), map.ranges[i] * np.sin(map.angle_min + i * map.angle_increment), 0.0) for i in range(len(map.ranges))], dtype=[('x', np.float32),('y', np.float32), ('z', np.float32)])
        
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
    map_to_pointcloud2 = MapToPointCloud2()
    rclpy.spin(map_to_pointcloud2)
    map_to_pointcloud2.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
