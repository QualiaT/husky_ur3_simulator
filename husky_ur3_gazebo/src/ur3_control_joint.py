#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import math
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist

def ur3_control(q1,q2,q3,q4,q5,q6):
    pub1 = rospy.Publisher('/husky/joint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/husky/joint2_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/husky/joint3_position_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/husky/joint4_position_controller/command', Float64, queue_size=10)
    pub5 = rospy.Publisher('/husky/joint5_position_controller/command', Float64, queue_size=10)
    pub6 = rospy.Publisher('/husky/joint6_position_controller/command', Float64, queue_size=10)
    pub0 = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)


    husky_speed = Twist()
    
    husky_speed.linear.x = 0.2
    husky_speed.angular.z = 0.2
    



    rospy.init_node('husky_ur3_joint_control', anonymous=True)
    

    r = rospy.Rate(4)

    pub0.publish(husky_speed)
    pub1.publish(q1)
    pub2.publish(q2)
    pub3.publish(q3)
    pub4.publish(q4)
    pub5.publish(q5)
    pub6.publish(q6)                
    r.sleep()

if __name__ == '__main__':

    ur3_control(1.57,-2.27,1.93,-1.19,-1.57,0)
    print "done"
    ur3_control(0,0,0,0,0,0)
    print "done"
    ur3_control(1.57,0,0,0,0,0)
    print "done"
    ur3_control(1.57,-2.27,1.93,-1.19,-1.57,0)


