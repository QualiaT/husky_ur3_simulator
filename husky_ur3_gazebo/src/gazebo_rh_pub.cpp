/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Float64.h>

ros::Publisher g_rh_r2_joint_pub;
ros::Publisher g_rh_l1_joint_pub;
ros::Publisher g_rh_l2_joint_pub;


void rhJointCallback(const std_msgs::Float64::ConstPtr& msg)
{
  std_msgs::Float64 grip_joint_msg, grip_joint_msg_2;

  grip_joint_msg.data = msg->data;
  if (grip_joint_msg.data > 1.05)
    grip_joint_msg.data = 1.05;
  grip_joint_msg_2.data = msg->data * (1.0 / 1.1);

  g_rh_r2_joint_pub.publish(grip_joint_msg_2);
  g_rh_l1_joint_pub.publish(grip_joint_msg);
  g_rh_l2_joint_pub.publish(grip_joint_msg_2);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "gazebo_gripper_publisher");
  ros::NodeHandle nh("~");

  // 210927 YSW: Removed gripper namespace
  /*
  g_rh_r2_joint_pub = nh.advertise<std_msgs::Float64>("/rh_p12_rn/rh_r2_position/command", 0);
  g_rh_l1_joint_pub = nh.advertise<std_msgs::Float64>("/rh_p12_rn/rh_l1_position/command", 0);
  g_rh_l2_joint_pub = nh.advertise<std_msgs::Float64>("/rh_p12_rn/rh_l2_position/command", 0);

  
  */

  g_rh_r2_joint_pub = nh.advertise<std_msgs::Float64>("/rh_r2_position/command", 0);
  g_rh_l1_joint_pub = nh.advertise<std_msgs::Float64>("/rh_l1_position/command", 0);
  g_rh_l2_joint_pub = nh.advertise<std_msgs::Float64>("/rh_l2_position/command", 0);

  ros::Subscriber rh_joint_sub = nh.subscribe("/rh_p12_rn_position/command", 5, rhJointCallback);

  ros::spin();

  return 0;
}
