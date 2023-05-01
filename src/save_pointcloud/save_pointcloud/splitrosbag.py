import csv
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from your_package.msg import CustomMsg

class CsvToBagNode(Node):
    def __init__(self):
        super().__init__('csv_to_bag_node')
        self.publisher = self.create_publisher(CustomMsg, 'csv_data', 10)
        self.timer = self.create_timer(0.1, self.publish_csv_data)
        
        self.csv_file = open('data.csv', 'r')
        self.csv_reader = csv.reader(self.csv_file)
        
    
    def publish_csv_data(self):
        row = next(self.csv_reader, None)
        if row is None:
            self.get_logger().info('End of CSV file reached.')
            self.timer.cancel()
            self.csv_file.close()
            return
        
        
        timestamp = float(row[0])
        header = Header()
        header.stamp = self.get_clock().from_time(timestamp)
        msg = CustomMsg()
        msg.header = header
        msg.field1 = float(row[1])
        msg.field2 = float(row[2])
        msg.field3 = float(row[3])
        self.publisher.publish(msg)
        self.get_logger().info('Published message with timestamp %f', timestamp)


def main(args=None):
    rclpy.init(args=args)
    node = CsvToBagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()