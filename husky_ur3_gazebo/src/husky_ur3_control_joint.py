#!/usr/bin/env python
# license removed for brevity
import rospy, math, time
import roslib; roslib.load_manifest('husky_ur3_gazebo')
from std_msgs.msg import Float64
import trajectory_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist

def ur3_control():
    rospy.init_node('husky_ur3_joint_control', anonymous=True)

    print rospy.get_rostime().to_sec()
    while rospy.get_rostime().to_sec() == 0.0:
        time.sleep(0.1)
        print rospy.get_rostime().to_sec()

    pub0 = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
    pub1 = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)



    ur3_ctrl = JointTrajectory()
    ur3_ctrl.header.frame_id = "husky::base_link"

    ur3_ctrl.header.stamp = rospy.Time.now()

    ur3_ctrl.joint_names.append("husky::shoulder_pan_joint")
    ur3_ctrl.joint_names.append("husky::shoulder_lift_joint")
    ur3_ctrl.joint_names.append("husky::elbow_joint")
    ur3_ctrl.joint_names.append("husky::wrist_1_joint")
    ur3_ctrl.joint_names.append("husky::wrist_2_joint")
    ur3_ctrl.joint_names.append("husky::wrist_3_joint")

    n = 1500
    dt = 0.01
    rps = 0.05
    for i in range (n):
        p = JointTrajectoryPoint()
        theta = rps*2.0*math.pi*i*dt
        x1 = -0.5*math.sin(2*theta)
        x2 =  0.5*math.sin(1*theta)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        ur3_ctrl.points.append(p)

        # set duration
        ur3_ctrl.points[i].time_from_start = rospy.Duration.from_sec(dt)
        rospy.loginfo("test: angles[%d][%f, %f]",n,x1,x2)


    husky_speed = Twist()
    husky_speed.linear.x = 0.2
    husky_speed.angular.z = 0.2
    



    

   # r = rospy.Rate(4)

    #pub0.publish(husky_speed)
    pub1.publish(ur3_ctrl)
    rospy.spin()            
    #r.sleep()

if __name__ == '__main__':
    try:
        ur3_control()
    except rospy.ROSInterruptException: pass

