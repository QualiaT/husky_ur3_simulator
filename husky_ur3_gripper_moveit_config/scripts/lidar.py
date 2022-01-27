#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def callback(msg):
    print(len(msg.ranges))
    print(msg.ranges[0])
    print(msg.ranges[605 // 2])
    print(msg.ranges[604])


rospy.init_node("check_lidar")
sub = rospy.Subscriber("/scan", LaserScan, callback)  # We subscribe to the laser's topic
pub = rospy.Publisher("/cmd_vel", Twist)
move = Twist()

rospy.spin()
