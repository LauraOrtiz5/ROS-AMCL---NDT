#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer, TransformException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time, Duration

class TransformScanNode(Node):

    def __init__(self):
        super().__init__('transform_scan_node')
        # self.tf_buffer = Buffer()
        self.tf_buffer = Buffer(cache_time=rclpy.time.Duration(seconds=20))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        )
        self.scan_buffer = []
        self.transf_scan_publisher = self.create_publisher(
            LaserScan,
            '/transf_scan',
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        )
        self.get_logger().info('Transform Scan node started')

    def scan_callback(self, scan_msg):
        current_time = self.get_clock().now()
        try:
            transform_msg = self.tf_buffer.lookup_transform(
                'odom',
                scan_msg.header.frame_id,
                scan_msg.header.stamp,
                timeout=rclpy.time.Duration(seconds=1)
            )
        except TransformException as e:
            self.get_logger().error('Exception: ' + str(e))
            # Store the scan data in the buffer
            self.scan_buffer.append(scan_msg)
            return
        
        # If there is buffered data, process it now
        if self.scan_buffer:
            self.process_buffered_scans()

        transformed_scan = LaserScan()
        transformed_scan.header.frame_id = 'odom'
        transformed_scan.header.stamp = current_time.to_msg()
        transformed_scan.angle_min = scan_msg.angle_min
        transformed_scan.angle_max = scan_msg.angle_max
        transformed_scan.angle_increment = scan_msg.angle_increment
        transformed_scan.time_increment = scan_msg.time_increment
        transformed_scan.scan_time = scan_msg.scan_time
        transformed_scan.range_min = scan_msg.range_min
        transformed_scan.range_max = scan_msg.range_max
        transformed_scan.ranges = []
        transformed_scan.intensities = []

        for i in range(len(scan_msg.ranges)):
            point = [scan_msg.ranges[i], 0.0, 0.0]  # set y and z to 0.0
            while True:
                try: 
                    transformed_point = self.tf_buffer.trasnform(point, 'odom', scan_msg.header.stamp, timeout=rclpy.time.Duration(seconds=1))
                except TransformException as e:
                    self.get_logger().error('Exception: ' + str(e))
                    continue
                break
        
            transformed_point = transform_msg.transform.translation
            transformed_point.x += point[0] * transform_msg.transform.rotation.x
            transformed_point.y += point[0] * transform_msg.transform.rotation.y
            transformed_point.z += point[0] * transform_msg.transform.rotation.z
            transformed_scan.ranges.append(transformed_point.x)
            transformed_scan.intensities.append(0.0)

        self.transf_scan_publisher.publish(transformed_scan)

    def process_buffered_scans(self):
        i = 0
        while i < len(self.scan_buffer):
            scan_msg = self.scan_buffer[i]
            try:
                self.tf_buffer.lookup_transform(
                    'odom', scan_msg.header.frame_id, scan_msg.header.stamp,
                    tiemout=rclpy.time.Duration(seconds=1))
            except TransformException:
                # This scan message is still not transformable, leave it in the buffer
                i += 1
            else:
                # This scan message can now be transformed, so remove it from the buffer and process it
                del self.scan_buffer[i]
                self.scan_callback(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    transform_scan_node = TransformScanNode()
    rclpy.spin(transform_scan_node)
    transform_scan_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

