#include "image_undistort/interpolator_tester.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "interpolator_tester_node");
  ros::NodeHandle nh, private_nh("~");

  image_undistort::Tester tester(nh, private_nh);

  ros::spin();

  return 0;
}