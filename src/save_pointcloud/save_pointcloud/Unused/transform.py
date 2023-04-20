import subprocess
import rclpy
from rclpy.node import Node

class Showing(Node):

    def show(Node):
        subprocess.call(["python","combi_columns.py", "scan_callback"])
        print(table_scan)

def main():
    rclpy.init()
    node = Showing()
    rclpy.spin(node)

if __name__=='__main__':
    main()