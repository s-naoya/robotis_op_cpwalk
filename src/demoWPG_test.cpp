// This node has ROBOTIS-OP2 walk by only Capture Point.
// Using "cpgen" and "robotis_op_utility/robotis_body.h".
// After this node running, need operate this node
//   that use topic "/robotis_op/cmd_ctrl" and "/robotis_op/cmd_pos".
//
// /robotis_op/cmd_ctrl : sensor_msgs/Int32.h
//   1: walking stop, 2: walking start, 3: walking finish, 4: neutral
// /robotis_op/cmd_pos  : geometry_msgs/Pose2D.h
//   next step distance <x[m], y[m], theta[degree]>

#include <ros/ros.h>

// #include <cpgen/cpgen.h>

#include <robotis_op_utility/robotis_body.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>

#include <unordered_map>

#include<iostream>
#include<fstream>

// RobotisBody class's jointstate update and return CoM and Link pose.
class RobotisBodyUpdater {
 public:
  RobotisBodyUpdater() {
    ros::NodeHandle nh;
    jointstate = nh.subscribe("/robotis_op/joint_states", 1000,
                              &RobotisBodyUpdater::jsCallback, this);
    ros::Duration(0.5).sleep();  // wait to call jsCallback
    ROS_INFO("[RobotisBodyUpdater] activate");
  }

  // Update JointState and CoM, and publish to topic
  void jsCallback(const sensor_msgs::JointState& msg) {
    js = msg;
  }

  void moveInitPosition() { body.moveInitPosition(js); }
  void demoWPG() {body.demoWPG();}

 private:
  robotis::RobotisBody body;
  ros::Subscriber jointstate;
  sensor_msgs::JointState js;
};


int main(int argc, char** argv) {
  // initialize this node
  ros::init(argc, argv, "walking_pattern_generator");
  ros::NodeHandle nh;

  // for JointState subscriber and more
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // initialize RobotisBodyUpdater
  RobotisBodyUpdater body;
  body.moveInitPosition();
  ros::Duration(1.0).sleep();  // wait to call jsCallback

  ros::Rate rate(200);  // TODO!: use param
  while(ros::ok()) {
    body.demoWPG();
    rate.sleep();
  }
  ROS_INFO("CP walk finish");
  return 0;
}