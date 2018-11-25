// This node has ROBOTIS-OP2 walk by only Capture Point.
// Using "cpgen" and "conoid/robot.h".
// After this node running, need operate this node
//   that use topic "/robotis_op/cmd_ctrl" and "/robotis_op/cmd_pos".
//
// /robotis_op/cmd_ctrl : sensor_msgs/Int32.h
//   1: walking stop, 2: walking start, 3: walking finish, 4: neutral
// /robotis_op/cmd_pos  : geometry_msgs/Pose2D.h
//   next step distance <x[m], y[m], theta[degree]>

#include <fstream>
#include <iostream>
#include <map>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <conoid/robot.h>
#include <cpgen/cpgen.h>


void cmdPosCallback(const geometry_msgs::Pose2D& msg);
void cmdCtrlCallback(const std_msgs::Int32& msg);
void jsCallback(const sensor_msgs::JointState& msg);
void logging(double sec, const con::rl& spl, const con::RobotPtr& r);

enum walk_control_command { stop = 1, start = 2, finish = 3, neutral = 4 };
walk_control_command wcc;
cp::Vector3 land_pos;

con::RobotPtr robotis;
std::map<std::string, int> joint_id;

int main(int argc, char** argv) {
  // ros initialize
  ros::init(argc, argv, "walking_pattern_generator");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // setup Robot model
  // TODO! get wrl model file path by rosparam
  robotis = new con::Robot();
  if (!robotis->loadModel(
          "/home/naoya/catkin_ws/src/robotis_op_cpwalk/vrml/robotis_op.wrl")) {
    return 0;
  }
  const int dof = robotis->numJoints();
  robotis->update(std::vector<double>(dof, 0.0));
  const std::string leg_joint_name[2][2] = {{"j_pelvis_r", "j_ankle2_r"},
                                            {"j_pelvis_l", "j_ankle2_l"}};
  robotis->setFootLinks(
      leg_joint_name[con::right][0], leg_joint_name[con::right][1],
      leg_joint_name[con::left][0], leg_joint_name[con::left][1]);
  for (int i = 0; i < dof; ++i) {
    joint_id[robotis->joint(i)->name()] = i;
  }
  // setup publisher
  std::vector<ros::Publisher> js_pubs;
  for (int i = 0; i < dof; ++i) {
    std::string pub_name = "/robotis_op/" + robotis->joint(i)->name() +
                           "_position_controller/command";
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(pub_name, 50);
    js_pubs.push_back(pub);
  }

  // setup subscribers
  ros::Subscriber js_sub =
      nh.subscribe("/robotis_op/joint_states", 1000, jsCallback);
  ros::Subscriber cmd_ctrl_sub =
      nh.subscribe("/robotis_op/cmd_ctrl", 1000, cmdCtrlCallback);
  ros::Subscriber cmd_pos_sub =
      nh.subscribe("/robotis_op/cmd_pos", 1000, cmdPosCallback);

  // move half-sit pose
  std::vector<double> init_angles(dof, 0.0);
  std::vector<double> half_sit_angles(dof);
  std::map<std::string, double> half_sit_map;
  half_sit_map["j_thigh2_r"] = -0.55;
  half_sit_map["j_tibia_r"] = 1.1;
  half_sit_map["j_ankle1_r"] = 0.55;
  half_sit_map["j_thigh2_l"] = 0.55;
  half_sit_map["j_tibia_l"] = -1.1;
  half_sit_map["j_ankle1_l"] = -0.55;
  for (int i = 0; i < dof; ++i) {
    const std::string joint_name = robotis->joint(i)->name();
    half_sit_angles[i] = half_sit_map.find(joint_name) != half_sit_map.end()
                             ? half_sit_map[joint_name]
                             : 0.0;
  }
  // sequence move between init_angles to half_sit_angles
  const double dt = 0.005, half_sit_time = 2.0;  // [second]
  double roop_time = 0.0;
  ros::Rate rate(1 / dt);
  ROS_INFO("[cpwalk] start moving half-sit for %f second", half_sit_time);
  while (ros::ok()) {
    double ratio = roop_time / half_sit_time;
    for (int i = 0; i < dof; ++i) {
      std_msgs::Float64 msg_angle;
      double angle = init_angles[i] * (1 - ratio) + half_sit_angles[i] * ratio;
      msg_angle.data = angle;
      js_pubs[i].publish(msg_angle);
      robotis->joint(i)->q() = angle;
    }
    robotis->calcForwardKinematics();
    roop_time += dt;
    if (roop_time >= half_sit_time) {
      break;
    }
    double sec = ros::Time::now().toSec();
    logging(sec, con::null, robotis);
    rate.sleep();
  }
  ROS_INFO("[cpwalk] finish moving half-sit");

  // initialize cpgen
  cp::Vector3 wp_com = robotis->calcCenterOfMass();
  const cp::Affine3 init_waist_pose =
      cp::Translation3(robotis->rootLink()->p()) * robotis->rootLink()->R();
  const cp::Affine3 init_leg_pose[2] = {
      cp::Affine3(
          cp::Translation3(robotis->link(leg_joint_name[cp::right][1])->p()) *
          robotis->link(leg_joint_name[cp::right][1])->R()),
      cp::Affine3(
          cp::Translation3(robotis->link(leg_joint_name[cp::left][1])->p()) *
          robotis->link(leg_joint_name[cp::left][1])->R())};
  const cp::Quat base2leg[2] = {cp::Quat::Identity(), cp::Quat::Identity()};
  const double end_cp_offset[2] = {0.1, 0.4};
  const double single_sup_time = 0.5, double_sup_time = 0.2;
  const double cog_h = wp_com.z(), leg_h = 0.02;
  cp::cpgen cpgen;
  cpgen.initialize(wp_com, init_waist_pose, init_leg_pose, base2leg,
                   end_cp_offset, dt, single_sup_time, double_sup_time, cog_h,
                   leg_h);

  cp::Pose wp_leg_pose[2] = {cp::affine2pose(init_leg_pose[cp::right]),
                             cp::affine2pose(init_leg_pose[cp::left])};
  cp::Quat wp_waist_rot = cp::Quat(init_waist_pose.linear());
  wcc = neutral;
  while (ros::ok()) {
    double sec = ros::Time::now().toSec();

    switch (wcc) {
      case stop: {
        cpgen.stop();
        break;
      }
      case start: {
        cpgen.start();
        break;
      }
      case finish: {
        ros::shutdown();
        break;
      }
      case neutral: {
        break;
      }
    }
    wcc = neutral;

    cpgen.setLandPos(land_pos);
    cpgen.getWalkingPattern(&wp_com, &wp_waist_rot, &wp_leg_pose[cp::right],
                            &wp_leg_pose[cp::left]);
    cp::rl swl = cpgen.getSwingleg();
    con::rl spl = swl == cp::right ? con::left : con::right;
    con::Vector3 ref_feet_pos[2] = {wp_leg_pose[cp::right].p(),
                                    wp_leg_pose[cp::left].p()};
    con::Matrix3 ref_feet_rot[2] = {
        wp_leg_pose[cp::right].q().toRotationMatrix(),
        wp_leg_pose[cp::left].q().toRotationMatrix()};
    if (!robotis->calcIKforWalking(spl, wp_com, wp_waist_rot.toRotationMatrix(),
                                   ref_feet_pos, ref_feet_rot)) {
      ROS_ERROR("[cpwalk] Failed IK");
    }

    for (int i = 0; i < dof; ++i) {
      std_msgs::Float64 angle;
      angle.data = robotis->joint(i)->q();
      js_pubs[i].publish(angle);
    }
    logging(sec, spl, robotis);
    rate.sleep();
  }
}

void logging(double sec, const con::rl& spl, const con::RobotPtr& r) {
  static std::ofstream ofs("cpwalk_log.csv", std::ios::out);
  static std::string sep = ",";
  static bool is_first = true;
  if (is_first) {
    ofs << "time" << sep << "sup leg";
    ofs << sep << "com_x" << sep << "com_y" << sep << "com_z";
    ofs << sep << "waist_r" << sep << "waist_p" << sep << "waist_ya";
    ofs << sep << "r_foot_x" << sep << "r_foot_y" << sep << "r_foot_z";
    ofs << sep << "r_foot_r" << sep << "r_foot_p" << sep << "r_foot_ya";
    ofs << sep << "l_foot_x" << sep << "l_foot_y" << sep << "l_foot_z";
    ofs << sep << "l_foot_r" << sep << "l_foot_p" << sep << "l_foot_ya";
    ofs << std::endl;
    is_first = false;
  }
  ofs << sec << sep << spl;
  ofs << sep << r->calcCenterOfMass().x() << sep << r->calcCenterOfMass().y()
      << sep << r->calcCenterOfMass().z();
  ofs << sep << cp::mat2rpy(r->rootLink()->R()).x() << sep
      << cp::mat2rpy(r->rootLink()->R()).y() << sep
      << cp::mat2rpy(r->rootLink()->R()).z();
  ofs << sep << r->link("j_ankle2_r")->p().x() << sep
      << r->link("j_ankle2_r")->p().y() << sep
      << r->link("j_ankle2_r")->p().z();
  ofs << sep << cp::mat2rpy(r->link("j_ankle2_r")->R()).x() << sep
      << cp::mat2rpy(r->link("j_ankle2_r")->R()).y() << sep
      << cp::mat2rpy(r->link("j_ankle2_r")->R()).z();
  ofs << sep << r->link("j_ankle2_l")->p().x() << sep
      << r->link("j_ankle2_l")->p().y() << sep
      << r->link("j_ankle2_l")->p().z();
  ofs << sep << cp::mat2rpy(r->link("j_ankle2_l")->R()).x() << sep
      << cp::mat2rpy(r->link("j_ankle2_l")->R()).y() << sep
      << cp::mat2rpy(r->link("j_ankle2_l")->R()).z();
  ofs << std::endl;
}

void cmdCtrlCallback(const std_msgs::Int32& msg) {
  int32_t t = msg.data;
  switch (t) {
    case 1: {
      wcc = stop;
      break;
    }
    case 2: {
      wcc = start;
      break;
    }
    case 3: {
      wcc = finish;
      break;
    }
    case 4: {
      wcc = neutral;
      break;
    }
    default: {
      wcc = neutral;
      break;
    }
  }
}

void cmdPosCallback(const geometry_msgs::Pose2D& msg) {
  land_pos << msg.x, msg.y, con::deg2rad(msg.theta);
}

void jsCallback(const sensor_msgs::JointState& msg) {
  // please use this callback if robotis can be going to output servo value
  // for (int i = 0, dof = robotis->numJoints(); i < dof; ++i) {
  //   robotis->joint(joint_id[msg.name[i]])->q() = msg.position[i];
  // }
  // robotis->calcForwardKinematics();
}