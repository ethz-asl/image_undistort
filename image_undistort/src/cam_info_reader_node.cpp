#include <image_undistort/cam_info_reader.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cam_info_reader_node");

  ros::NodeHandle nh, private_nh("~");

  ROS_INFO("Started cam info reader node.");

  CamInfoReader cam_info_reader(nh, private_nh);

  ros::spin();

  return 0;
}