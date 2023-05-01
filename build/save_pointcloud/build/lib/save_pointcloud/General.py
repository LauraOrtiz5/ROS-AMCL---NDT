import rospy
import tf2_ros
import sensor_msgs.msg
import tf2_geometry_msgs
from tf2_geometry_msgs import PointStamped
from tf.transformations import *

if __name__ == "__main__":
    rospy.init_node('robot_tf_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pub = rospy.Publisher('/transf_scan', sensor_msgs.msg.LaserScan, queue_size=10)

    def scan_callback(scan_msg):
        # Create a new LaserScan message to store the transformed data
        transformed_scan = sensor_msgs.msg.LaserScan()
        
        # Set the header of the transformed_scan message
        transformed_scan.header.stamp = rospy.Time.now()
        transformed_scan.header.frame_id = "base_link"
        
        # Transform each point in the scan message to the "base_link" frame
        for i, range_val in enumerate(scan_msg.ranges):
            laser_point = PointStamped()
            laser_point.header.stamp = rospy.Time.now()
            laser_point.header.frame_id = "base_laser"
            laser_point.point.x = range_val
            laser_point.point.y = 0.2
            laser_point.point.z = 0
            
            try:
                transform = tfBuffer.lookup_transform("base_link", "base_laser", rospy.Time(0), rospy.Duration(1.0))
                transformed_point = tf2_geometry_msgs.do_transform_point(laser_point, transform)
                transformed_scan.ranges.append(transformed_point.point.x)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(e)
        
        # Publish the transformed scan message
        pub.publish(transformed_scan)

    # Subscribe to the scan topic and register the callback function
    rospy.Subscriber('/scan', sensor_msgs.msg.LaserScan, scan_callback)

    # Spin the node to prevent it from exiting
    rospy.spin()
