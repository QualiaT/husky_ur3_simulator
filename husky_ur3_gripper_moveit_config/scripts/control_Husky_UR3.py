#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Jan 10 11:16:00 2014
@author: Sam Pfeiffer
Snippet of code on how to send a navigation goal and how to get the current robot position in map
Navigation actionserver: /move_base/goal
Type of message: move_base_msgs/MoveBaseActionGoal
Actual robot pose topic: /amcl_pose
Type of message: geometry_msgs/PoseWithCovarianceStamped
"""
import sys
import rospy
import copy, math
from math import pi, radians, degrees, atan2, sqrt
from moveit_commander import MoveGroupCommander, RobotCommander 
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
from nav_msgs.msg import Odometry








#GROUP_NAME_GRIPPER = "NAME OF GRIPPER"
roscpp_initialize(sys.argv)
rospy.init_node('control_Husky_UR3', anonymous=True)
robot = RobotCommander()
scene = PlanningSceneInterface()



##모바일 파트 관련 변수
x = 0.0
y = 0.0 
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)




## 매니퓰레이터 변수 선언

group_name = "ur3_manipulator"
move_group = MoveGroupCommander(group_name)
FIXED_FRAME = 'world'

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               DisplayTrajectory,
                                               queue_size=20)









#매니퓰레이터 제어

joint_goal = move_group.get_current_joint_values()


mobile_joints = [-pi/2, 0.5]
joint_goal_list = [pi*90/180, pi*-130/180 , pi*111/180, pi*-68/180, pi*-90/180, 0] #home pose

goal_x_dist = mobile_joints[1] #meter
angle_to_goal = mobile_joints[0]
goal_distance = 0
r.sleep()
while abs(theta-angle_to_goal) >0.01:
    
    if abs(theta-angle_to_goal) < 0.2:
        speed.angular.z = 0.02
    else:
        speed.linear.x = 0.0
        speed.angular.z = 0.2
    
    pub.publish(speed)
    r.sleep()
    print "theta:",theta,"goal_th:",angle_to_goal

init_x = x
init_y = y

while abs(goal_x_dist - goal_distance) >0.01:

    current_dist = abs(sqrt((x-init_x) ** 2 + (y-init_y) ** 2))
    goal_distance = current_dist

    if abs(goal_x_dist - goal_distance) < 0.1:
        speed.linear.x = 0.02
        speed.angular.z = 0
    else:
        speed.linear.x = 0.3
        speed.angular.z = 0
    
    pub.publish(speed)
    r.sleep()
    goal_distance = current_dist
    print "distance:",goal_distance

joint_goal[0] = joint_goal_list[0]
joint_goal[1] = joint_goal_list[1]
joint_goal[2] = joint_goal_list[2]
joint_goal[3] = joint_goal_list[3]
joint_goal[4] = joint_goal_list[4]
joint_goal[5] = joint_goal_list[5]
move_group.go(joint_goal, wait=True)



mobile_joints = [-pi/2, 0.5]
joint_goal_list = [1.74601981851980,	-2.41258905754319,	1.09760409640119,	-1.71170200248478,	-1.64091964834470,	-0.0121189757710538] #home pose

goal_x_dist = mobile_joints[1] #meter
angle_to_goal = mobile_joints[0]
goal_distance = 0
r.sleep()
while abs(theta-angle_to_goal) >0.01:
    
    if abs(theta-angle_to_goal) < 0.2:
        speed.angular.z = 0.02
    else:
        speed.linear.x = 0.0
        speed.angular.z = 0.2
    
    pub.publish(speed)
    r.sleep()
    print "theta:",theta,"goal_th:",angle_to_goal

init_x = x
init_y = y

while abs(goal_x_dist - goal_distance) >0.01:

    current_dist = abs(sqrt((x-init_x) ** 2 + (y-init_y) ** 2))
    goal_distance = current_dist

    if abs(goal_x_dist - goal_distance) < 0.1:
        speed.linear.x = 0.02
        speed.angular.z = 0
    else:
        speed.linear.x = 0.3
        speed.angular.z = 0
    
    pub.publish(speed)
    r.sleep()
    goal_distance = current_dist
    print "distance:",goal_distance

joint_goal[0] = joint_goal_list[0]
joint_goal[1] = joint_goal_list[1]
joint_goal[2] = joint_goal_list[2]
joint_goal[3] = joint_goal_list[3]
joint_goal[4] = joint_goal_list[4]
joint_goal[5] = joint_goal_list[5]
move_group.go(joint_goal, wait=True)



mobile_joints = [-pi/2, 0.5]
joint_goal_list = [1.40145883148585,	-1.95948636426188,	1.19698107691592,	-1.72091255567091,	-1.53601915790332,	0.0433483209748009] #pick pose

goal_x_dist = mobile_joints[1] #meter
angle_to_goal = mobile_joints[0]
goal_distance = 0
r.sleep()
while abs(theta-angle_to_goal) >0.01:
    
    if abs(theta-angle_to_goal) < 0.2:
        speed.angular.z = 0.02
    else:
        speed.linear.x = 0.0
        speed.angular.z = 0.2
    
    pub.publish(speed)
    r.sleep()
    print "theta:",theta,"goal_th:",angle_to_goal

init_x = x
init_y = y

while abs(goal_x_dist - goal_distance) >0.01:

    current_dist = abs(sqrt((x-init_x) ** 2 + (y-init_y) ** 2))
    goal_distance = current_dist

    if abs(goal_x_dist - goal_distance) < 0.1:
        speed.linear.x = 0.02
        speed.angular.z = 0
    else:
        speed.linear.x = 0.3
        speed.angular.z = 0
    
    pub.publish(speed)
    r.sleep()
    goal_distance = current_dist
    print "distance:",goal_distance

joint_goal[0] = joint_goal_list[0]
joint_goal[1] = joint_goal_list[1]
joint_goal[2] = joint_goal_list[2]
joint_goal[3] = joint_goal_list[3]
joint_goal[4] = joint_goal_list[4]
joint_goal[5] = joint_goal_list[5]
move_group.go(joint_goal, wait=True)


mobile_joints = [-pi/2, 0.5]
joint_goal_list = [1.39535030784060,	-1.97737109028856,	1.21555131713052,	-1.68764464622035,	-1.53577309578646,	0.0428461912103362] #pick pose

goal_x_dist = mobile_joints[1] #meter
angle_to_goal = mobile_joints[0]
goal_distance = 0
r.sleep()
while abs(theta-angle_to_goal) >0.01:
    
    if abs(theta-angle_to_goal) < 0.2:
        speed.angular.z = 0.02
    else:
        speed.linear.x = 0.0
        speed.angular.z = 0.2
    
    pub.publish(speed)
    r.sleep()
    print "theta:",theta,"goal_th:",angle_to_goal

init_x = x
init_y = y

while abs(goal_x_dist - goal_distance) >0.01:

    current_dist = abs(sqrt((x-init_x) ** 2 + (y-init_y) ** 2))
    goal_distance = current_dist

    if abs(goal_x_dist - goal_distance) < 0.1:
        speed.linear.x = 0.02
        speed.angular.z = 0
    else:
        speed.linear.x = 0.3
        speed.angular.z = 0
    
    pub.publish(speed)
    r.sleep()
    goal_distance = current_dist
    print "distance:",goal_distance

joint_goal[0] = joint_goal_list[0]
joint_goal[1] = joint_goal_list[1]
joint_goal[2] = joint_goal_list[2]
joint_goal[3] = joint_goal_list[3]
joint_goal[4] = joint_goal_list[4]
joint_goal[5] = joint_goal_list[5]
move_group.go(joint_goal, wait=True)





move_group.stop()
















 