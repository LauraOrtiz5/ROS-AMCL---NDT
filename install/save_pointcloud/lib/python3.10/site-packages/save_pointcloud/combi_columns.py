import os
import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
import pandas as pd
import math
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turtlesim.srv import Spawn

# Create an empty pointcloud
point_cloud_data = []

# Create an empty table
table_scan = pd.DataFrame(columns=['timestamp','x','y','z'])
table_tf = pd.DataFrame(columns=['timestamp','tx','ty','tz','qx','qy','qz','qw'])

class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter('target_frame', 'odom').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # List of dictionaries to store the transformation info
        self.transform_list = []

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

        # Initialize variables to keep track of failures
        self.num_consecutive_failures = 0
        self.max_num_failures = 5

    def on_timer(self):
        global table_tf
        # Store frame names in variables that will be used to compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'base_scan'
        try:
            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
            # Extract the transformation parameters
            transform_dict = {'timestamp': t.header.stamp.sec + t.header.stamp.nanosec*1e-9,
                              'tx': t.transform.translation.x,
                              'ty': t.transform.translation.y,
                              'tz': t.transform.translation.z,
                              'qx': t.transform.rotation.x,
                              'qy': t.transform.rotation.y,
                              'qz': t.transform.rotation.z,
                              'qw': t.transform.rotation.w}
            
            # Add transformation info to the list
            table_tf = table_tf.append(transform_dict, ignore_index=True)
            print("\n-------------------------------------------------------------------------------------------------\n")
            print(table_tf)
            print("\n-------------------------------------------------------------------------------------------------\n")
            table_tf.to_csv('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/tables/table_tf.csv', index=False)
            
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            self.num_consecutive_failures += 1
            
            # Stop node after too many consecutve failures (i.e. 5)
            if self.num_consecutive_failures >= self.max_num_failures:
                self.get_logger().info(f'Stopping node due to {self.max_num_failures} consecutive failures')
                self.timer.cancel()
                self.destroy_node()
                rclpy.shutdown()

def scan_callback(scan):
    global point_cloud_data, table_scan

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

    # Add timestamp and points to table
    new_scan = {'timestamp': scan.header.stamp.sec + scan.header.stamp.nanosec*1e-9,
                'x': x,
                'y': y,
                'z': z}
    table_scan = table_scan.append(new_scan, ignore_index=True)

    print("\n-------------------------------------------------------------------------------------------------\n")
    print(table_scan)
    print("\n-------------------------------------------------------------------------------------------------\n")
    table_scan.to_csv('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/tables/table_scan.csv', index=False)
    

    return table_scan
 
def main():
    rclpy.init()
    
    node = FrameListener()

    qos_profile = QoSProfile(depth=10)
    sub = node.create_subscription(LaserScan, '/scan', scan_callback, qos_profile=qos_profile)
    rclpy.spin(node)

if __name__=='__main__':
    main()
