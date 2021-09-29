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
import threading
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



##모바일 파트 관련 변수 선언
x = 0.0
y = 0.0 
theta = 0.0

## 매니퓰레이터 변수 선언

group_name = "ur3_manipulator"
move_group = MoveGroupCommander(group_name)
FIXED_FRAME = 'world'

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               DisplayTrajectory,
                                               queue_size=20)

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def move_base(a,b):             #x,y좌표를 받아와서 그곳으로 platform을 움직이는 코드 theta는 x와 y좌표에 의해 정해짐.(원점기준)
    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    speed = Twist()
    r = rospy.Rate(4)

    goal = Point()
    goal.x = a
    goal.y = b
    goal_angle = atan2(goal.y,goal.x)

    while abs(goal.x-x) >0.1 or abs(goal.y-y) >0.1 or abs(goal_angle-theta) >0.1 : #가까의 범위가 0.1이내로 들어오면 break.
        inc_x = goal.x -x
        inc_y = goal.y -y

        angle_to_goal = atan2(inc_y, inc_x)

        if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
            if(angle_to_goal -theta)>0 and (angle_to_goal -theta)<0.1:   # 범위 이내로 들어오면 속도를 매우 줄여서 목표점을 지나쳐버리는 일이 없도록함.
                speed.linear.x = 0.0                
                speed.angular.z = -0.02
            elif(angle_to_goal-theta)<0 and (angle_to_goal -theta)>-0.1:
                speed.linear.x = 0.0
                speed.angular.z = 0.02

        else:
            speed.linear.x = 0.3
            speed.angular.z = 0.0
            if abs(goal.x-x)<0.1 and abs(goal.y-y)<0.1:  # 범위 이내로 들어오면 속도를 매우 줄여서 목표점을 지나쳐버리는 일이 없도록함.
                speed.linear.x = 0.01
                speed.angular.z = 0.0
        print goal.x-x, goal.y-y, goal_angle-theta
        pub.publish(speed)
        r.sleep() 

def move_joint(q1,q2,q3,q4,q5,q6):
    joint_goal = move_group.get_current_joint_values()

    mobile_joints = [-pi/3, 0.5]
    joint_goal_list = [q1,q2,q3,q4,q5,q6] 

    #매니퓰레이터 관절 value 설정
    joint_goal[0] = joint_goal_list[0]
    joint_goal[1] = joint_goal_list[1]
    joint_goal[2] = joint_goal_list[2]
    joint_goal[3] = joint_goal_list[3]
    joint_goal[4] = joint_goal_list[4]
    joint_goal[5] = joint_goal_list[5]
    #제어시작
    move_group.go(joint_goal, wait=True)

def move_mobile_manipulator(a,b,q1,q2,q3,q4,q5,q6):
    move_joint(pi*90/180, pi*-130/180 , pi*111/180, pi*-68/180, pi*-90/180, 0) #UR3 home pose

    
    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    speed = Twist()
    r = rospy.Rate(4)

    goal = Point()
    goal.x = a
    goal.y = b

    inc_x = goal.x -x
    inc_y = goal.y -y
    angle_to_goal = atan2(inc_y, inc_x)


    while abs(goal.x-x) >0.1 or abs(goal.y-y) >0.1 or abs(angle_to_goal-theta) >0.1 : #가까의 범위가 0.1이내로 들어오면 break.
        inc_x = goal.x -x
        inc_y = goal.y -y

        angle_to_goal = atan2(inc_y, inc_x)

        if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
            if abs(angle_to_goal - theta) < 0.3:        # 0.5이내로 들어오면 속도를 매우 줄여서 목표점을 지나쳐버리는 일이 없도록함.
                speed.angular.z = 0.03
                speed.linear.x = 0.0

        
        else:
            speed.linear.x = 0.2
            speed.angular.z = 0.0
            if abs(goal.x-x) <0.5 and abs(goal.y-y)<0.5:
                speed.angular.x = 0.05
                speed.angular.z = 0.0

        print goal.x-x, goal.y-y, angle_to_goal-theta

        pub.publish(speed)
        r.sleep()    

    
    move_joint(q1,q2,q3,q4,q5,q6) #매니퓰레이터 조인트



def move_base_thread(a,b):
        thread=threading.Thread(target=move_base(a,b))
        thread.daemon=True
        thread.start()

def move_joint_thread(q1,q2,q3,q4,q5,q6):
        thread=threading.Thread(target=move_joint(q1,q2,q3,q4,q5,q6))
        thread.daemon=True
        thread.start()        






if __name__=='__main__':
    
    move_mobile_manipulator(2,2,pi/2,0,0,0,0,0)
    move_mobile_manipulator(1.1,1.1,pi/2,0,0,0,0,0)
    #move_mobile_manipulator(1.2,1.2,pi/2,0,0,0,0,0)
    #move_mobile_manipulator(1.3,1.3,pi/2,0,0,0,0,0)
    #move_mobile_manipulator(1.4,1.4,pi/2,0,0,0,0,0)
#

    #move_mobile_manipulator(2,2,pi/2,0,0,0,0,0)
    #move_mobile_manipulator(-2,-2,pi/2,0,0,0,0,0)
    #move_mobile_manipulator(0.25,0.25,pi/2,0,0,0,0,0)    

  

























 