import os
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import rospy
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import pandas as pd
import transformations as tf
import math
import geometry_msgs.msg
import tf2_ros
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turtlesim.srv import Spawn
from rclpy.duration import Duration
from rosgraph_msgs.msg import Log
# from rclpy.callback_groups import CallbackGroup
# from rclpy.executors import SingleThreadedExecutor

# Create an empty pointcloud
point_cloud_data = []

# Create an empty table
table_scan = pd.DataFrame(columns=['timestamp','x','y','z'])
table_tf = pd.DataFrame(columns=['timestamp','tx','ty','tz','qx','qy','qz','qw'])

def scan_callback(scan):
    #print("Received scan data:", scan)
    global point_cloud_data, table_scan, table_tf

    ranges = np.array(scan.ranges)
    angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

    # Convert any possible NaN or inf values into 0
    ranges[np.isnan(ranges)]=0
    angles[np.isnan(angles)]=0
    ranges[np.isinf(ranges)]=0
    angles[np.isinf(angles)]=0

    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    z = np.zeros(len(ranges))
    points = np.vstack((x, y, z)).T

    # Get the latest transform from /base_scan to /odom
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, Node)
    def transform_points(points, transform_matrix):
        ones = np.ones((points.shape[0],1))
        points_homogeneous = np.hstack((points, ones))
        points_transformed_homogeneous = np.dot(transform_matrix, points_homogeneous.T)
        return points_transformed_homogeneous[:3,:].T
    
    points_transformed = np.array([])
    try:
        transform_stamped = tf_buffer.lookup_transform('odom', 'base_scan', rospy.Time(0), rospy.Duration(1.0))
        transform_matrix = tf2_ros.transformations.transform_matrix(transform_stamped.transform)
        points_transformed = transform_points(points, transform_matrix)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
        rospy.logwarn(f"Could not transform point cloud {ex}")


    #  points_transformed = np.array([])
    # try:
    #     tf_buffer = Buffer()
    #     # tf_listener= TransformListener(tf_buffer)
    #     # tf_listener.waitForTransform('odom','base_scan',scan.header.stamp,rospy.Duration(5))
    
    #     # if tf_listener.canTransform('odom','base_scan',scan.header.stamp):
    #     # transform_stamped = tf_buffer.lookup_transform('odom', 'base_scan', scan.header.stamp, timeout=Duration(seconds=1))
    #     transform_stamped = tf_buffer.lookup_transform('odom', 'base_scan', scan.header.stamp, timeout=Duration(seconds=1))
    #     print(transform_stamped)
    #     transform_matrix = TransformStamped2Matrix(transform_stamped)
    #     points_transformed = transform_points(points, transform_matrix)
    # except TransformException as ex:
    #     print(f'Could not transform point cloud: {ex}')

    # Append the transformed points to the list
    point_cloud_data.extend(points_transformed.tolist())

    with open ('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/pcd_files/Transf.pcd','wb') as f:
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

def TransformStamped2Matrix(transform_stamped):
    """
    Convert a TransformStamped message to a 4x4 transformation matrix.
    """
    translation = transform_stamped.transform.translation
    rotation = transform_stamped.transform.rotation
    trans_matrix = tf.translation_matrix((translation.x, translation.y, translation.z))
    rot_matrix = tf.quaternion_matrix((rotation.x, rotation.y, rotation.z, rotation.w))
    transform_matrix = np.dot(trans_matrix, rot_matrix)
    return transform_matrix

def transform_points(points, transform_matrix):
    """
    Apply a 4x4 transformation matrix to a set of 3D points.
    """
    hom_points = np.hstack((points, np.ones((points.shape[0], 1))))
    transformed_hom_points = np.dot(transform_matrix, hom_points.T).T
    transformed_points = transformed_hom_points[:, :3]
    return transformed_points

class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter('target_frame', 'odom').get_parameter_value().string_value
        
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # List of dictionaries to store the transformation info
        self.transform_list = []

        # Call on_timer function every second
        # self.timer_callback_group = CallbackGroup()
        # self.timer = self.create_timer(1.0, self.on_timer, callback_group = self.timer_callback_group)
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
            
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            self.num_consecutive_failures += 1
            
            # Stop node after too many consecutve failures (i.e. 5)
            if self.num_consecutive_failures >= self.max_num_failures:
                self.get_logger().info(f'Stopping node due to {self.max_num_failures} consecutive failures')
                self.timer.cancel()
                self.destroy_node()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    node = FrameListener()

    qos_profile = QoSProfile(depth=10)
    # qos_profile = QoSProfile(
    # reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    # history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    # depth=10)
    sub = node.create_subscription(LaserScan, '/scan', scan_callback, qos_profile=qos_profile)

    # executor = SingleThreadedExecutor()
    # executor.add_node(node)
    # executor.spin()

    rclpy.spin(node)

if __name__=='__main__':
    main()
