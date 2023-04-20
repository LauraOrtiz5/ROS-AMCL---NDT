import math
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turtlesim.srv import Spawn


table_tf = []

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
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'base_scan'
        try:
            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
            # Extract the transformation parameters
            transform_dict = {'timestamp': t.header.stamp,
                              'from_frame': t.header.frame_id,
                              'to_frame': t.child_frame_id,
                              'translation': (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
                              'rotation': (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)}
            # Add transformation info to the list
            self.transform_list.append(transform_dict)
            self.get_logger().info(f'CTransform {to_frame_rel} to {from_frame_rel} is: {transform_dict}')
            table_tf.append(transform_dict)
            print("\n-------------------------------------------------------------------------------------------------\n")
            print(table_tf)
            print("\n-------------------------------------------------------------------------------------------------\n")
            self.num_consecutive_failures = 0
            
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            self.num_consecutive_failures += 1
            
            # Stop node after too many consecutve failures (i.e. 5)
            if self.num_consecutive_failures >= self.max_num_failures:
                self.get_logger().info(f'Stopping node due to {self.max_num_failures} consecutive failures')
                self.timer.cancel()
                self.destroy_node()
                rclpy.shutdown()


def main():
    rclpy.init()
    
    node = FrameListener()
    rclpy.spin(node)

    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass

    # Print transformation    
    #rclpy.shutdown()
