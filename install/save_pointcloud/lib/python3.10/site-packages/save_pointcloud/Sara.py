#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose

rospy.init_node('initial_pose_publisher')

pose_pub = rospy.Publisher('/initial_pose', Pose, queue_size=10)

rate = rospy.Rate(10) # 10 Hz

while not rospy.is_shutdown():
    pose_msg = Pose()
    pose_msg.position.x = 0.0
    pose_msg.position.y = 0.0
    pose_msg.position.z = 0.0
    pose_msg.orientation.x = 0.0
    pose_msg.orientation.y = 0.0
    pose_msg.orientation.z = 0.0
    pose_msg.orientation.w = 1.0

    pose_pub.publish(pose_msg)

    if pose_pub.get_num_connections() > 0:
        rospy.loginfo("Initial pose published")
        break

    rate.sleep()

rospy.spin()