#include <image_undistort/image_undistort.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_undistort_node");

  ros::NodeHandle nh, nh_private("~"), nh_input("input"), nh_output("output");

  ROS_INFO("Started image undistort node.");

  ImageUndistort image_undistort(nh, nh_input, nh_output, nh_private);

  ros::spin();

  return 0;
}