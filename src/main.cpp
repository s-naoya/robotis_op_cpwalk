// This node has ROBOTIS-OP2 walk by only Capture Point.
// Using "cpgen" and "robotis_op_utility/robotis_body.h".
// After this node running, need operate this node
//   that use topic "/robotis_op/cmd_ctrl" and "/robotis_op/cmd_pos".
//
// /robotis_op/cmd_ctrl : sensor_msgs/Int32.h
//   1: walking stop, 2: walking start 3: walking finish
// /robotis_op/cmd_pos  : geometry_msgs/Pose2D.h
//   next step distance <x[m], y[m], theta[degree]>

#include <ros/ros.h>

#include <cpgen/cpgen.h>

#include <robotis_op_utility/robotis_body.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>

#include <unordered_map>

enum walk_control_command { stop = 1, start = 2, finish = 3, neutral = 4 };
walk_control_command wcc;
cp::Vector3 land_pos;

// RobotisBody class's jointstate update and return CoM and Link pose.
class RobotisBodyUpdater {
 public:
  RobotisBodyUpdater() {
    ros::NodeHandle nh;
    jointstate = nh.subscribe("/robotis_op/joint_states", 1000,
                              &RobotisBodyUpdater::jsCallback, this);
    com_pub =
        nh.advertise<geometry_msgs::Point>("/robotis_op/center_of_mass", 50);
    com = cp::Vector3::Zero();
    ros::Duration(0.5).sleep();  // wait to call jsCallback
    ROS_INFO("[RobotisBodyUpdater] activate");
  }

  // Update JointState and CoM, and publish to topic
  void jsCallback(const sensor_msgs::JointState& msg) {
    body.update(msg);
    com = body.calcCenterOfMass();
    js = msg;

    geometry_msgs::Point p;
    p.x = com[0];
    p.y = com[1];
    p.z = com[2];
    com_pub.publish(p);
  }

  void moveInitPosition() { body.moveInitPosition(js); }

  bool calcLegIK(const cp::Vector3& com, const cp::Affine3& right_leg,
                 const cp::Affine3& left_leg) {
    std::unordered_map<std::string, double> joint_values;

    if (body.calkIKofWalkingMotion(com, right_leg, left_leg, joint_values)) {
      body.publishJointCommand(joint_values);
      return true;
    } else {
      // moveInitPosition();
      return false;
    }
  }

  cp::Affine3 getLinkAffine(std::string name) { return body.link_affine[name]; }

  cp::Vector3 com;

 private:
  robotis::RobotisBody body;
  ros::Subscriber jointstate;
  ros::Publisher com_pub;
  sensor_msgs::JointState js;
};

// convert cp::Pose(declaration in "eigen_types.h") to geometry_msgs::Pose
geometry_msgs::Pose epose2gpose(cp::Pose b_pose) {
  geometry_msgs::Pose r_pose;
  r_pose.position.x = b_pose.p().x();
  r_pose.position.y = b_pose.p().y();
  r_pose.position.z = b_pose.p().z();
  r_pose.orientation.x = b_pose.q().x();
  r_pose.orientation.y = b_pose.q().y();
  r_pose.orientation.z = b_pose.q().z();
  r_pose.orientation.w = b_pose.q().w();
  return r_pose;
}

// convert cp::Vector3(Eigen's Vector3d) to geometry_msgs::Vector3
geometry_msgs::Vector3 evector2gvector(cp::Vector3 b_vec) {
  geometry_msgs::Vector3 r_vec;
  r_vec.x = b_vec[0];
  r_vec.y = b_vec[1];
  r_vec.z = b_vec[2];
  return r_vec;
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
  land_pos << msg.x, msg.y, cp::deg2rad(msg.theta);
}

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

  // initialize cpgen
  std::array<cp::Affine3d, 2> init_leg_pos{body.getLinkAffine("MP_ANKLE2_R"),
                                           body.getLinkAffine("MP_ANKLE2_L")};
  cp::Quaternion base2r(cp::AngleAxis(cp::deg2rad(90.0), cp::Vector3::UnitX()));
  cp::Quaternion base2l(
      cp::AngleAxis(cp::deg2rad(-90.0), cp::Vector3::UnitX()));
  std::array<cp::Quaternion, 2> base2leg{base2r, base2l};
  // for returned walking pattern
  cp::Vector3 wp_com(0.0, 0.0, body.com.z());
  cp::cpgen cpgen(wp_com, init_leg_pos, base2leg);
  cp::Pose wp_right_leg_pos = cpgen.setInitLandPos(init_leg_pos[0]);
  cp::Pose wp_left_leg_pos  = cpgen.setInitLandPos(init_leg_pos[1]);

  // setup control command subscriber
  wcc = neutral;
  ros::Subscriber cmd_ctrl_sub =
      nh.subscribe("/robotis_op/cmd_ctrl", 1000, cmdCtrlCallback);

  // set reference landing position
  ros::Subscriber cmd_pos_sub =
      nh.subscribe("/robotis_op/cmd_pos", 1000, cmdPosCallback);
  land_pos << 0.0, 0.0, cp::deg2rad(0.0);

  ros::Rate rate(200);  // TODO!: use param
  while (ros::ok()) {
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
    cpgen.getWalkingPattern(body.com, &wp_com, &wp_right_leg_pos,
                            &wp_left_leg_pos);
    body.calcLegIK(wp_com, wp_right_leg_pos.affine(), wp_left_leg_pos.affine());
    rate.sleep();
  }
  ROS_INFO("CP walk finish");
  return 0;
}