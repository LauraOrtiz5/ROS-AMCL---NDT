import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPoseSubscriber(Node):
    def __init__(self):
        super().__init__('initial_pose_subscriber')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.callback,
            10)
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10)
        self.initial_pose = None

    def callback(self, msg):
        if self.initial_pose is None:
            self.initial_pose = msg
            self.publisher_.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    initial_pose_subscriber = InitialPoseSubscriber()
    rclpy.spin(initial_pose_subscriber)
    initial_pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
