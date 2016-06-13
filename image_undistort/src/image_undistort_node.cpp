#include <image_undistort/image_undistort.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_undistort_node");

  ros::NodeHandle nh, private_nh("~");

  ROS_INFO("Started image undistort node.");

  ImageUndistort image_undistort(nh, private_nh);

  ros::spin();

  return 0;
}