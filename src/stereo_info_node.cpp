#include <image_undistort/stereo_info.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_info_node");

  ros::NodeHandle nh, nh_private("~");

  ROS_INFO("Started stereo info node.");

  StereoInfo stereo_info(nh, nh_private);

  ros::spin();

  return 0;
}