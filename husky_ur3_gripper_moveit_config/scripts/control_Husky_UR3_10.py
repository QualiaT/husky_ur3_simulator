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

from threading import Thread
import multiprocessing
from multiprocessing import Process
from math import pi, radians, degrees, atan2, sqrt
from moveit_commander import MoveGroupCommander, RobotCommander 
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_commander.conversions import pose_to_list, list_to_pose
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
from ar_track_alvar_msgs.msg import AlvarMarkers

import numpy as np
from numpy import linalg


import numpy as np
import tf.transformations as tf
from math import *
import cmath
from geometry_msgs.msg import Pose, Quaternion

"""Kinematics Docstring
   This python script provides solution of forward kinematics and 
   inverse kinematics for Universal Robot UR3/5/10.
"""

# Congifuration
# Select the robot type.
# UR10 for 'UR10'
# UR5 for 'UR5'
# UR3 for 'UR3'

ROBOT = 'UR3'


# DH Parameter

if ROBOT == 'UR10':

    # d (unit: mm)
    d1 = 0.1273
    d2 = d3 = 0
    d4 = 0.163941
    d5 = 0.1157
    d6 = 0.0922

    # a (unit: mm)
    a1 = a4 = a5 = a6 = 0
    a2 = -0.612
    a3 = -0.5723

elif ROBOT == 'UR5':

    # d (unit: mm)
    d1 = 0.089159 
    d2 = d3 = 0
    d4 = 0.10915
    d5 = 0.09465
    d6 = 0.0823

    # a (unit: mm)
    a1 = a4 = a5 = a6 = 0
    a2 = -0.425
    a3 = -0.39225

elif ROBOT == 'UR3':

    # d (unit: mm)
    d1 = 0.1519 
    d2 = d3 = 0
    d4 = 0.11235
    d5 = 0.08535
    d6 = 0.0819

    # a (unit: mm)
    a1 = a4 = a5 = a6 = 0
    a2 = -0.24365
    a3 = -0.21325


# List type of D-H parameter
# Do not remove these
d = np.array([d1, d2, d3, d4, d5, d6]) # unit: mm
a = np.array([a1, a2, a3, a4, a5, a6]) # unit: mm
alpha = np.array([pi/2, 0, 0, pi/2, -pi/2, 0]) # unit: radian


# Auxiliary Functions

def ur2ros(ur_pose):
    """Transform pose from UR format to ROS Pose format.
    Args:
        ur_pose: A pose in UR format [px, py, pz, rx, ry, rz] 
        (type: list)
    Returns:
        An HTM (type: Pose).
    """

    # ROS pose
    ros_pose = Pose()

    # ROS position
    ros_pose.position.x = ur_pose[0]
    ros_pose.position.y = ur_pose[1]
    ros_pose.position.z = ur_pose[2]

    # Ros orientation
    angle = sqrt(ur_pose[3] ** 2 + ur_pose[4] ** 2 + ur_pose[5] ** 2)
    direction = [i / angle for i in ur_pose[3:6]]
    np_T = tf.rotation_matrix(angle, direction)
    np_q = tf.quaternion_from_matrix(np_T)
    ros_pose.orientation.x = np_q[0]
    ros_pose.orientation.y = np_q[1]
    ros_pose.orientation.z = np_q[2]
    ros_pose.orientation.w = np_q[3]
    
    return ros_pose


def ros2np(ros_pose):
    """Transform pose from ROS Pose format to np.array format.
    Args:
        ros_pose: A pose in ROS Pose format (type: Pose)
    Returns:
        An HTM (type: np.array).
    """

    # orientation
    np_pose = tf.quaternion_matrix([ros_pose.orientation.x, ros_pose.orientation.y, \
                                    ros_pose.orientation.z, ros_pose.orientation.w])
    
    # position
    np_pose[0][3] = ros_pose.position.x
    np_pose[1][3] = ros_pose.position.y
    np_pose[2][3] = ros_pose.position.z

    return np_pose


def np2ros(np_pose):
    """Transform pose from np.array format to ROS Pose format.
    Args:
        np_pose: A pose in np.array format (type: np.array)
    Returns:
        An HTM (type: Pose).
    """

    # ROS pose
    ros_pose = Pose()

    # ROS position
    ros_pose.position.x = np_pose[0, 3]
    ros_pose.position.y = np_pose[1, 3]
    ros_pose.position.z = np_pose[2, 3]

    # ROS orientation 
    np_q = tf.quaternion_from_matrix(np_pose)
    ros_pose.orientation.x = np_q[0]
    ros_pose.orientation.y = np_q[1]
    ros_pose.orientation.z = np_q[2]
    ros_pose.orientation.w = np_q[3]

    return ros_pose


def select(q_sols, q_d, w=[1]*6):
    """Select the optimal solutions among a set of feasible joint value 
       solutions.
    Args:
        q_sols: A set of feasible joint value solutions (unit: radian)
        q_d: A list of desired joint value solution (unit: radian)
        w: A list of weight corresponding to robot joints
    Returns:
        A list of optimal joint value solution.
    """

    error = []
    for q in q_sols:
        error.append(sum([w[i] * (q[i] - q_d[i]) ** 2 for i in range(6)]))
    
    return q_sols[error.index(min(error))]


def HTM(i, theta):
    """Calculate the HTM between two links.
    Args:
        i: A target index of joint value. 
        theta: A list of joint value solution. (unit: radian)
    Returns:
        An HTM of Link l w.r.t. Link l-1, where l = i + 1.
    """

    Rot_z = np.matrix(np.identity(4))
    Rot_z[0, 0] = Rot_z[1, 1] = cos(theta[i])
    Rot_z[0, 1] = -sin(theta[i])
    Rot_z[1, 0] = sin(theta[i])

    Trans_z = np.matrix(np.identity(4))
    Trans_z[2, 3] = d[i]

    Trans_x = np.matrix(np.identity(4))
    Trans_x[0, 3] = a[i]

    Rot_x = np.matrix(np.identity(4))
    Rot_x[1, 1] = Rot_x[2, 2] = cos(alpha[i])
    Rot_x[1, 2] = -sin(alpha[i])
    Rot_x[2, 1] = sin(alpha[i])

    A_i = Rot_z * Trans_z * Trans_x * Rot_x
	    
    return A_i


# Forward Kinematics

def fwd_kin(theta, i_unit='r', o_unit='n'):
    """Solve the HTM based on a list of joint values.
    Args:
        theta: A list of joint values. (unit: radian)
        i_unit: Output format. 'r' for radian; 'd' for degree.
        o_unit: Output format. 'n' for np.array; 'p' for ROS Pose.
    Returns:
        The HTM of end-effector joint w.r.t. base joint
    """

    T_06 = np.matrix(np.identity(4))

    if i_unit == 'd':
        theta = [radians(i) for i in theta]
    
    for i in range(6):
        T_06 *= HTM(i, theta)

    if o_unit == 'n':
        return T_06
    elif o_unit == 'p':
        return np2ros(T_06)


# Inverse Kinematics

def inv_kin(p, q_d, i_unit='r', o_unit='r'):
    """Solve the joint values based on an HTM.
    Args:
        p: A pose.
        q_d: A list of desired joint value solution 
             (unit: radian).
        i_unit: Output format. 'r' for radian; 'd' for degree.
        o_unit: Output format. 'r' for radian; 'd' for degree.
    Returns:
        A list of optimal joint value solution.
    """

    # Preprocessing
    if type(p) == Pose: # ROS Pose format
        T_06 = ros2np(p)
    elif type(p) == list: # UR format
        T_06 = ros2np(ur2ros(p))

    if i_unit == 'd':
        q_d = [radians(i) for i in q_d]

    # Initialization of a set of feasible solutions
    theta = np.zeros((8, 6))
 
    # theta1
    P_05 = T_06[0:3, 3] - d6 * T_06[0:3, 2]
    phi1 = atan2(P_05[1], P_05[0])
    phi2 = acos(d4 / sqrt(P_05[0] ** 2 + P_05[1] ** 2))
    theta1 = [pi / 2 + phi1 + phi2, pi / 2 + phi1 - phi2]
    theta[0:4, 0] = theta1[0]
    theta[4:8, 0] = theta1[1]
  
    # theta5
    P_06 = T_06[0:3, 3]
    theta5 = []
    for i in range(2):
        theta5.append(acos((P_06[0] * sin(theta1[i]) - P_06[1] * cos(theta1[i]) - d4) / d6))
    for i in range(2):
        theta[2*i, 4] = theta5[0]
        theta[2*i+1, 4] = -theta5[0]
        theta[2*i+4, 4] = theta5[1]
        theta[2*i+5, 4] = -theta5[1]
  
    # theta6
    T_60 = np.linalg.inv(T_06)
    theta6 = []
    for i in range(2):
        for j in range(2):
            s1 = sin(theta1[i])
            c1 = cos(theta1[i])
            s5 = sin(theta5[j])
            theta6.append(atan2((-T_60[1, 0] * s1 + T_60[1, 1] * c1) / s5, (T_60[0, 0] * s1 - T_60[0, 1] * c1) / s5))
    for i in range(2):
        theta[i, 5] = theta6[0]
        theta[i+2, 5] = theta6[1]
        theta[i+4, 5] = theta6[2]
        theta[i+6, 5] = theta6[3]

    # theta3, theta2, theta4
    for i in range(8):  
        # theta3
        T_46 = HTM(4, theta[i]) * HTM(5, theta[i])
        T_14 = np.linalg.inv(HTM(0, theta[i])) * T_06 * np.linalg.inv(T_46)
        P_13 = T_14 * np.array([[0, -d4, 0, 1]]).T - np.array([[0, 0, 0, 1]]).T
        if i in [0, 2, 4, 6]:
            theta[i, 2] = -cmath.acos((np.linalg.norm(P_13) ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)).real
            theta[i+1, 2] = -theta[i, 2]
        # theta2
        theta[i, 1] = -atan2(P_13[1], -P_13[0]) + asin(a3 * sin(theta[i, 2]) / np.linalg.norm(P_13))
        # theta4
        T_13 = HTM(1, theta[i]) * HTM(2, theta[i])
        T_34 = np.linalg.inv(T_13) * T_14
        theta[i, 3] = atan2(T_34[1, 0], T_34[0, 0])       

    theta = theta.tolist()

    # Select the most close solution
    q_sol = select(theta, q_d)

    # Output format
    if o_unit == 'r': # (unit: radian)
        return q_sol
    elif o_unit == 'd': # (unit: degree)
        return [degrees(i) for i in q_sol]






def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def ar_position(msg):

    global ar_x
    global ar_y
    global ar_z
    ar_topic_pose = Pose()
    try:    
        ar_topic_pose.position.x = msg.markers[0].pose.pose.position.x
        ar_topic_pose.position.y = msg.markers[0].pose.pose.position.y
        ar_topic_pose.position.z = msg.markers[0].pose.pose.position.z
        ar_x = ar_topic_pose.position.x
        ar_y = ar_topic_pose.position.y
        ar_z = ar_topic_pose.position.z
    except:
        return





def jmove_to_pose_goal(x_p,y_p,z_p,x_o,y_o,z_o,w_o):
    pose_goal = Pose()
    pose_goal.position.x = x_p
    pose_goal.position.y = y_p
    pose_goal.position.z = z_p
    pose_goal.orientation.x = x_o
    pose_goal.orientation.y = y_o
    pose_goal.orientation.z = z_o
    pose_goal.orientation.w = w_o
    move_group.set_pose_target(pose_goal)
    move_group.go(wait=False)
    #tf_display_position = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z]      
    #tf_display_orientation = [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
    #ii = 0
    #while ii < 5:
    #    ii += 1
    #    br = tf.TransformBroadcaster()
    #    br.sendTransform(
    #        tf_display_position,
    #        tf_display_orientation,
    #        rospy.Time.now(),
    #        "Target_pose",
    #        "base_link")
    #    rospy.sleep(1)
        
def jmove_to_joint_goal(joint_goal):
    move_group.go(joint_goal, wait=False)


def move_Joint(q1,q2,q3,q4,q5,q6):
    joint_goal = move_group.get_current_joint_values()

    joint_goal_list = [q1,q2,q3,q4,q5,q6] 

    #매니퓰레이터 관절 value 설정
    joint_goal[0] = joint_goal_list[0]
    joint_goal[1] = joint_goal_list[1]
    joint_goal[2] = joint_goal_list[2]
    joint_goal[3] = joint_goal_list[3]
    joint_goal[4] = joint_goal_list[4]
    joint_goal[5] = joint_goal_list[5]
    #제어시작
    move_group.go(joint_goal, wait=False)




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

    arrival_radius = 0.3

    while (goal.x-x)**2 + (goal.y-y)**2 >= arrival_radius**2 :
    #while abs(goal.x-x) >0.1 or abs(goal.y-y) >0.1 or abs(angle_to_goal-theta) >0.1 : #가까의 범위가 0.3이내로 들어오면 break.
        
        inc_x = goal.x -x
        inc_y = goal.y -y
        angle_to_goal = atan2(inc_y,inc_x)

        if abs(angle_to_goal - theta) > 5*pi/180:
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

        #print goal.x-x, goal.y-y, angle_to_goal-theta

        pub.publish(speed)
        #r.sleep() 

    final_angle_to_goal = 0
    while abs(final_angle_to_goal - theta) > 0.02:
        if abs(final_angle_to_goal - theta) > 0.3:
            speed.linear.x = 0
            speed.angular.z = 0.3
        else:
            speed.linear.x = 0
            speed.angular.z = 0.1           

        pub.publish(speed)
        r.sleep()           

    print 'mobile robot movement complete!'

    
    return x,y

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
    rospy.sleep(1)
    pose_target.position.x +=  a  # First move up (z)
    move_group.set_pose_target(pose_target)
    move_group.go(True)

def y_path_planner(c):
    pose_target = move_group.get_current_pose().pose
    rospy.sleep(1)
    pose_target.position.y +=  c  # First move up (z)
    move_group.set_pose_target(pose_target)
    move_group.go(True)      

def z_path_planner(b):
    pose_target = move_group.get_current_pose().pose
    rospy.sleep(1)
    pose_target.position.z +=  b  # First move up (z)
    move_group.set_pose_target(pose_target)
    move_group.go(True)  

    
def down_demo():
    move_Joint(1.57,-2.27,1.93,-1.19,-1.57,0) #down pose 
    print "Down demo is ready to start!, press enter..!"
    raw_input()
    z_path_planner(0.1)
    y_path_planner(-0.112)  
    x_path_planner(0.1)    
    z_path_planner(-0.2)
    rospy.sleep(3)
    print "Down demo complete!, Go to home pose..!"


def cartesian_path(x,y,z):
    scale = 1
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z += scale * 0.1  # First move up (z)
    wpose.position.y += scale * y  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * x  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z += scale * z  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

def Grasp_object(x_dir,y_dir,z_dir):

    current_mobile_x,current_mobile_y = move_base(x_dir-0.5,y_dir)
    #z_path_planner(0.1)
    print "Grasping is ready to start!, press enter..!"
    raw_input()
    curr_pose = move_group.get_current_pose().pose
    
    x_distance = current_mobile_x+curr_pose.position.x - x_dir  
    y_distance = current_mobile_y+curr_pose.position.y -  y_dir  
    z_distance = curr_pose.position.z -  z_dir  

    print curr_pose.position.x
    print 'x_dir =',x_dir,'y_dir=',y_dir,'z_dir=',z_dir

    print 'x =',x_distance,'y=',y_distance,'z=',z_distance
    #y_path_planner(-y_distance)  
    #x_path_planner(-x_distance)    
    #z_path_planner(-z_distance)
    plan,fraction  = cartesian_path(-x_distance, -y_distance, -z_distance)
    move_group.execute(plan, wait=True)

    rospy.sleep(3)
    (result_xyz,result_rot) = get_TF('/odom','ee_link')
    print 'xyz_result=',result_xyz[0],result_xyz[1],result_xyz[2]
    print "Grasping complete!, Go to home pose..!"
    
    



if __name__=='__main__':

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

    
    # custom 역기구학 test
    goal_pose = Pose()
    goal_pose.position.x = -0.0
    goal_pose.position.y = 0.2
    goal_pose.position.z = 0.4
    #goal_pose.orientation.x = 0.707
    #goal_pose.orientation.y = 0.0
    #goal_pose.orientation.z = 0.707    
    goal_pose.orientation.w = 1.0




    # ar마커 변수 선언
    ar_x = 0
    ar_y = 0
    ar_z = 0

    #move_Joint(1.57,-2.27,1.93,-1.19,-1.57,0) #down pose 
   # move_Joint(1.57,-2.27,1.93,-1.19,-1.57,0) #home pose
    #rospy.sleep(5)

    #rospy.sleep(5)







    # 사용자입력 : 목표 eef 위치 지정 (x,y,z,r,p,yaw)
    goal_eef = [0,0,0,0,0,0,1]
    goal_eef_quat = list_to_pose(goal_eef)




    #vector_x = ar_tr[0] - eef_tr[0]
    #vector_y = ar_tr[1] - eef_tr[1]

    #angle_to_goal = atan2(vector_y,vector_x)
    #distance_to_goal_eef = sqrt(vector_x**2 + vector_y**2)

    # ar 마커 및 eef pose data 선언.
    ar_pose = Pose()
    eef_pose = Pose()

    






    sub1 = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, ar_position)

    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    #rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

    speed = Twist()
    r = rospy.Rate(4)

    #목표eef의 방향으로 0.5미터 앞까지 모바일 플랫폼을 직진
    while not rospy.is_shutdown():

        #ar_tr,ar_rot = get_TF('base_link','ar_marker_0')
        #eef_tr,eef_rot = get_TF('base_link','rh_p12_rn_base')
        eef_poses = move_group.get_current_pose().pose

        vector_x = ar_x - eef_poses.position.x
        vector_y = ar_y - eef_poses.position.y
        vector_z = ar_z - eef_poses.position.z
        print vector_x
        #print eef_tr,eef_rot
        # AR마커 goal 지정 : 목표 eef 위치 지정
        ar_pose = Pose()
        ar_pose.position.x = ar_x
        ar_pose.position.y = ar_y
        ar_pose.position.z = ar_z
        ar_pose.orientation.x = 0.707
        ar_pose.orientation.y = 0.0
        ar_pose.orientation.z = 0.707
        ar_pose.orientation.w = 0.0
        
        virtual_goal = Pose()
        virtual_goal = ar_pose
        virtual_goal.position.x = ar_x-0.5

        if vector_x >= 0.25:
            speed.linear.x = 0.02
            speed.angular.z = 0.0

        elif vector_x <0.25:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
        
        
        #pub.publish(speed)


        #d_joint = move_group.get_current_joint_values()
        #sol = inv_kin(virtual_goal,d_joint)
        #move_Joint(pi/2,0,0,0,pi/2,0)
        th1 = Thread(target=jmove_to_pose_goal, args=(virtual_goal.position.x,virtual_goal.position.y,virtual_goal.position.z,virtual_goal.orientation.x,
                                                                virtual_goal.orientation.y,virtual_goal.orientation.z,virtual_goal.orientation.w))
        

        th1.start()
        th1.join()
        #jmove_to_pose_goal(virtual_goal)
        
        #print(sol)
        #jmove_to_joint_goal(sol)




 

    








    









  

























 