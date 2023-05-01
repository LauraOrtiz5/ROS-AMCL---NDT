import rclpy
import tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import time

point_cloud_data = []

class TransformBufferNode(Node):

    def __init__(self):
        super().__init__("transform_buffer_node")
        self.buffer = tf2_ros.Buffer()
        
        time.sleep(10)

        while not self.buffer.can_transform("odom", "base_scan", tf2_ros.Time()):
            self.get_logger().info("Waiting for transform from 'base_scan' to 'odom'...")
            rclpy.spin_once(self)

        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10)

    def scan_callback(self, scan_msg):
        try:
            transform = self.buffer.lookup_transform(
                "odom", scan_msg.header.frame_id, scan_msg.header.stamp, rclpy.time.Duration(seconds=1.0))
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f"Error buffering scan message: {ex}")
            return
        
        global point_cloud_data, table_scan, table_tf

        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        # Convert any possible NaN or inf values into 0
        ranges[np.isnan(ranges)]=0
        angles[np.isnan(angles)]=0
        ranges[np.isinf(ranges)]=0
        angles[np.isinf(angles)]=0

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros(len(ranges))
        points = np.vstack((x, y, z)).T
        
        points_transformed = np.array([])
        transform_stamped = self.tf_buffer.lookup_transform('odom', scan_msg.header.frame_id, scan_msg.header.stamp, rclpy.duration.Duration(seconds=1.0))

        translation = [transform_stamped.transform.translation.x, 
                           transform_stamped.transform.translation.y,
                           transform_stamped.transform.translation.z]
        rotation = [transform_stamped.transform.rotation.x,
                        transform_stamped.transform.rotation.y,
                        transform_stamped.transform.rotation.z,
                        transform_stamped.transform.rotation.w]
            
        R = np.array([[1 - 2 * (rotation[1]**2 + rotation[2]**2), 2 * (rotation[0] * rotation[1] - rotation[2] * rotation[3]), 2 * (rotation[0] * rotation[2] + rotation[1] * rotation[3]), translation[0]],
                      [2 * (rotation[0] * rotation[1] + rotation[2] * rotation[3]), 1 - 2 * (rotation[0]**2 + rotation[2]**2), 2 * (rotation[1] * rotation[2] - rotation[0] * rotation[3]), translation[1]],
                      [2 * (rotation[0] * rotation[2] - rotation[1] * rotation[3]), 2 * (rotation[1] * rotation[2] + rotation[0] * rotation[3]), 1 - 2 * (rotation[0]**2 + rotation[1]**2), translation[2]],
                      [0, 0, 0, 1]])
        
        # Apply the transform to the point cloud
        for point in points:
            points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
            points_transformed = np.dot(R, points_homogeneous.T).T[:, :3]


        # Append the transformed points to the list
        point_cloud_data.extend(points_transformed.tolist())

        with open ('/home/laura/ros2_ws/src/save_pointcloud/save_pointcloud/pcd_files/NDT_new.pcd','wb') as f:
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
    node = TransformBufferNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
