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



def move_linear_joint(linear_val):

    
    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    speed = Twist()
    r = rospy.Rate(4)
    i = 0

    while i<linear_val: 
        speed.linear.x = 0.02

        q1 = 1.57-i*0.01
        q2 = -2.27
        q3 = 1.93
        q4 = -1.19
        q5 = -1.57
        q6 = 0
        joint_goal = [q1,q2,q3,q4,q5,q6]
    

        pub.publish(speed)
        move_group.go(joint_goal, wait=True)

        r.sleep() 
        i = i+1

def move_linear(linear_val):

    
    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    speed = Twist()
    r = rospy.Rate(4)
    i = 0

    while i<linear_val: 
        speed.linear.x = 0.1

        pub.publish(speed)
        r.sleep() 
        i = i+1

def move_angular(angular_val):

    
    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    speed = Twist()
    r = rospy.Rate(4)

    i = 0

    while i<angular_val: 
        speed.angular.z = 0.1

        pub.publish(speed)
        r.sleep() 
        i = i+1




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

    move_Joint(1.57,-2.27,1.93,-1.19,-1.57,0) #home pose 
    move_linear_joint(50)










    









  

























 