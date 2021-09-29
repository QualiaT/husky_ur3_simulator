#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""

"""
import sys
import rospy
import copy, math
import threading
import time
import tf

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
from math import pi






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





def move_Joint(q1,q2,q3,q4,q5,q6):
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



def get_TF(a,b):
  end_flag = 0
  listener = tf.TransformListener()
  while end_flag ==0:
      try:
          (trans,rot) = listener.lookupTransform(a,b, rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      end_flag = 1

  return trans,rot
  
    
def move_ee(Px,Py,Pz,Rx,Ry,Rz,Rw):


  trans,rot = get_TF('/odom','/base_link')
  print('TF from odom to base link :',trans)
  x = Px-trans[0]
  y = Py-trans[1]
  z = Pz-trans[2]


  Ox = Rx
  Oy = Ry
  Oz = Rz-rot[2]
  Ow = Rw

  print 'real_planning_pose',x,y,z,Ox,Oy,Oz,Ow
  print "============ Generating plan 1"
  pose_target = Pose()
  pose_target.position.x = x
  pose_target.position.y = y
  pose_target.position.z = z
  pose_target.orientation.x = Ox
  pose_target.orientation.y = Oy
  pose_target.orientation.z = Oz
  pose_target.orientation.w = Ow
  move_group.set_pose_target(pose_target)
  move_group.go(True)
  print "============ plan 1 complete!"

  trans_1,rot_1 = get_TF('odom','/ee_link')
  print "============ ee pose : "
  print move_group.get_current_pose()
  print move_group.get_planning_frame()
  print 'odom_TF',trans_1,rot_1
  print "============"

def move_base(a,b):

    
    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    speed = Twist()
    r = rospy.Rate(4)

    goal = Point()
    goal.x = a
    goal.y = b

    arrival_radius = 0.1

    while (goal.x-x)**2 + (goal.y-y)**2 >= arrival_radius**2 :
    #while abs(goal.x-x) >0.1 or abs(goal.y-y) >0.1 or abs(angle_to_goal-theta) >0.1 : #가까의 범위가 0.3이내로 들어오면 break.
        
        inc_x = goal.x -x
        inc_y = goal.y -y
        angle_to_goal = atan2(inc_y,inc_x)

        if abs(angle_to_goal - theta) > 2*pi/180:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
            if abs(angle_to_goal - theta) < 5*pi/180:        # 0.5이내로 들어오면 속도를 매우 줄여서 목표점을 지나쳐버리는 일이 없도록함.
                speed.angular.z = 0.03
                speed.linear.x = 0.0

        
        else:
            speed.linear.x = 0.2
            speed.angular.z = 0.0
            if abs(goal.x-x) <0.3 and abs(goal.y-y)<0.3:    #x,y val이 0.3이내로 들어오면 속도 매우 줄임.
                speed.angular.x = 0.05
                speed.angular.z = 0.0

        print goal.x-x, goal.y-y, angle_to_goal-theta

        pub.publish(speed)
        r.sleep() 

def cartesian_path_planner(a,b,c):
    waypoints = []
    wpose = move_group.get_current_pose().pose
    wpose.position.z +=  a  # First move up (z)
    waypoints.append(copy.deepcopy(wpose))

    wpose = move_group.get_current_pose().pose
    wpose.position.x +=  b  # First move up (z)
    waypoints.append(copy.deepcopy(wpose))

    wpose = move_group.get_current_pose().pose
    wpose.position.y +=  c  # First move up (z)
    waypoints.append(copy.deepcopy(wpose))    

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.1,        # eef_step
                                       0.0)         # jump_threshold


def x_path_planner(a):
    pose_target = move_group.get_current_pose().pose
    pose_target.position.x +=  a  # First move up (z)
    move_group.set_pose_target(pose_target)
    move_group.go(True)

def y_path_planner(c):
    pose_target = move_group.get_current_pose().pose
    pose_target.position.y +=  c  # First move up (z)
    move_group.set_pose_target(pose_target)
    move_group.go(True)      

def z_path_planner(b):
    pose_target = move_group.get_current_pose().pose
    pose_target.position.z +=  b  # First move up (z)
    move_group.set_pose_target(pose_target)
    move_group.go(True)  

    
def down_demo():
    move_Joint(1.57,-2.27,1.93,-1.19,-1.57,0) #down pose 
    print "Down demo is ready to start!, press enter..!"
    raw_input()
    print "go up..!"    
    z_path_planner(0.1)
    print "go down..!"    
    z_path_planner(-0.1)
    rospy.sleep(2)
    print "Down demo complete!, Go to home pose..!"
    move_Joint(1.57,-2.27,1.93,-1.19,-1.57,0) #home pose 

def up_demo():

    move_Joint(1.57,-2.27,1.93,-1.19,1.57,0) #up pose 
    print "Up demo is ready to start!, press enter..!"
    raw_input()
    print "go up..!"  
    rospy.sleep(1)
    z_path_planner(-0.05)
    print "go down..!"    
    z_path_planner(0.1)
    rospy.sleep(1)
    print "up demo complete!, Go to home pose..!"
    move_Joint(1.57,-2.27,1.93,-1.19,-1.57,0) #home pose 

def left_demo():
    #move_Joint(1.57,-2.27,1.93,-1.19,1.57,0) #up pose 
    move_Joint(1.57,-2.27,1.93,-1.19,3.14,0) #left pose 
    print "Left demo is ready to start!, press enter..!"
    raw_input()
    print "go left..!"    
    #y_path_planner(0.1)
    print "go more left..!"    
    y_path_planner(-0.2)
    rospy.sleep(2)
    print "left demo complete!, Go to home pose..!"
    move_Joint(1.57,-2.27,1.93,-1.19,-1.57,0) #home pose 

def right_demo():
    move_Joint(1.57,-2.27,1.93,-1.19,0,0) #left pose 
    print "right demo is ready to start!, press enter..!"
    raw_input()
    print "go right..!"    
    y_path_planner(-0.1)
    print "go more right..!"    
    y_path_planner(0.2)
    rospy.sleep(2)
    print "right demo complete!, Go to home pose..!"
    move_Joint(1.57,-2.27,1.93,-1.19,-1.57,0) #home pose 



if __name__=='__main__':


    down_demo()
    up_demo()
    left_demo()
    right_demo()

    #move_Joint(1.57079632679490	,-1.57079632679490,	0,	0,	1.57079632679490,	0)                                                   
    #move_Joint(1.38187901932325	,0.594965748224829	,-1.84587120888068	,-0.259201159280024	,1.87922844334536	,-2.94403460825812)
    #move_Joint(1.49234992746732	,0.505575183819339	,-1.77749928330972	,-0.242572378864612	,2.19692733555951	,-3.04571339173395)
    #move_Joint(1.57882340366397	,0.392747674943758	,-1.68144316751832	,-0.294244456380595	,2.51322054731526	,3.12658213006687)
    #move_Joint(1.63317876240784	,0.285981577941942	,-1.57592439233013	,-0.472776683731581	,2.82134996965689	,2.93965970526083)
    #move_Joint(1.65163641729639	,0.202937041530315	,-1.05397766677144	,-2.29055198297394	,3.05995622779418	,1.57079637373908)
    #move_Joint(1.63317874347654	,-0.210429202660752	,-1.10151162936461	,-0.0669323613463442	,-2.82134998229054	,-2.93965974902066)
    #move_Joint(1.57882340096811	,-0.312808525716150	,-1.00858459420407	,-0.237259591096372	,-2.51322054719748	,-3.12658213005787)
    #move_Joint(1.49234990741608	,-1.25023043364197	,0.884986270249331	,-1.26185202847346	,-2.19692733997135	,3.04571337230654)
    #move_Joint(1.38187898379176	,-1.24138218321594	,0.770320266997513	,-1.16042411723514	,-1.87922844910760	,2.94403457922216)
    #move_Joint(1.26044305157603	,-1.22046262619007	,0.674930245153225	,-1.02526396256805	,-1.57079631704406	,2.83203570141689)
    #move_Joint(1.14354220642811	,-1.18364979561972	,0.595829299678309	,-0.849373924600574	,-1.28560567284022	,2.69592568050081)
    #move_Joint(1.04448751501302	,-1.11991084380463	,0.510072952763684	,-0.611447209983393	,-1.03738511473967	,2.51967923264632)
    #move_Joint(0.971282836139718	,-0.999323999902423	,0.355694706708534	,-0.267416174488388	,-0.838880551835716	,2.28194780565803)
    #move_Joint(-1.63477141196796	,-1.91133122955846	,-1.10151156835317	,-1.89146935483124	,2.82134997161739	,2.93965969942271)
    #move_Joint(-1.65322906921999	,-2.36794446930605	,-1.05397760826863	,0.280329388719014	,-3.05995623288959	,-1.57079636143505)
    #move_Joint(-1.63477141766021	,-1.98520583742980	,-1.57592434796521	,-0.959335599677850	,-2.82134996914004	,-2.93965967867828)
    #move_Joint(-1.58041604836938	,-2.00129787870086	,-1.68144318432848	,-1.01750430999134	,-2.51322054568872	,-3.12658213472640)
    #move_Joint(-1.49394257050708	,2.63601745038978	,1.77749927268549	,-2.89902024472062	,-2.19692733754341	,3.04571339154885)
    #move_Joint(-1.38347163064872	,2.54662686543240	,1.84587116794395	,-2.88239141905799	,-1.87922844943615	,2.94403456872028)
    #move_Joint(-1.26203572966623	,2.49325324698662	,1.87618006889619	,-2.79863698534015	,-1.57079632703325	,2.83203572396531)







    









  

























 