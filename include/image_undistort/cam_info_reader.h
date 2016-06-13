#ifndef CAM_INFO_READER_H
#define CAM_INFO_READER_H

#include <Eigen/Eigen>

// ros
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>


class CamInfoReader {
 public:
  CamInfoReader(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

 private:
  bool loadCamParams();

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  sensor_msgs::CameraInfo cam_info_;

  // publishers
  ros::Publisher cam_info_pub_;

};

#endif