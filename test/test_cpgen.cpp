#include <robotis_op_cpwalk/cpgen.h>

#include <gtest/gtest.h>

#include <robotis_op_utility/robotis_body.h>

#include <tf/transform_listener.h>
#include <tf/tf_eigen.h>

using namespace cp;

TEST(CPGenTest, CPtest) { SUCCEED(); }

TEST(CPGenTest, CoMtest) { SUCCEED(); }

TEST(CPGenTest, Legtest) {
  // init
  ros::NodeHandle nh;
  // tf init
  tf::TransformListener listener;
  tf::StampedTransform body2base_tf;
  Affine3 body2base;
  ros::Duration(2.0).sleep();
  try{
    listener.lookupTransform("base_link", "MP_BODY", ros::Time(0), body2base_tf);
    tf::transformEigenTFToEigen(body2base_tf.transform, body2base);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  // body init
  robotis::RobotisBody body(nh);
  Affine3 initleg[2] = {body.link_trans["MP_ANKLE2_R"],
                         body.link_trans["MP_ANKLE2_L"]};

  LegTrack legtrack;
  // legtrack.setup(5e-3, 0.5, 0.2, 0.01, );

  SUCCEED();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_cpgen");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}