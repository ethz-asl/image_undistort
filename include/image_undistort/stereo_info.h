#ifndef IMAGE_UNDISTORT_STEREO_INFO_H
#define IMAGE_UNDISTORT_STEREO_INFO_H

#include <image_transport/image_transport.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "image_undistort/camera_parameters.h"

namespace image_undistort {

// Default values

// queue size
constexpr int kQueueSize = 100;
// true to load input cam_info from ros parameters, false to get it from a
// cam_info topic
constexpr bool kDefaultInputCameraInfoFromROSParams = true;
// namespace to use when loading left camera parameters from ros params
const std::string kDefaultLeftCameraNamespace = "left_camera";
// namespace to use when loading right camera parameters from ros params
const std::string kDefaultRightCameraNamespace = "right_camera";
// the output focal length will be multiplied by this value. This has the effect
// of resizing the image by this scale factor.
constexpr double kDefaultScale = 1.0;
// if radtan distortion should be called radtan (ASL standard) or plumb_bob (ros
// standard)
constexpr bool kDefaultRenameRadtanPlumbBob = false;

class StereoInfo {
 public:
  StereoInfo(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void leftImageCallback(const sensor_msgs::ImageConstPtr& image_msg);

  void rightImageCallback(const sensor_msgs::ImageConstPtr& image_msg);

  void leftCameraInfoCallback(
      const sensor_msgs::CameraInfoConstPtr& camera_info);

  void rightCameraInfoCallback(
      const sensor_msgs::CameraInfoConstPtr& camera_info);

 private:
  void sendCameraInfo(const std_msgs::Header& header, const CameraSide& side,
                      const CameraIO& io);

  // nodes
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  image_transport::ImageTransport it_;

  // subscribers
  image_transport::Subscriber left_image_sub_;
  image_transport::Subscriber right_image_sub_;
  ros::Subscriber left_camera_info_sub_;
  ros::Subscriber right_camera_info_sub_;

  // publishers
  ros::Publisher left_camera_info_input_pub_;
  ros::Publisher right_camera_info_input_pub_;
  ros::Publisher left_camera_info_output_pub_;
  ros::Publisher right_camera_info_output_pub_;

  std::shared_ptr<StereoCameraParameters> stereo_camera_parameters_ptr_;

  int queue_size_;
  bool rename_radtan_plumb_bob_;
};
}

#endif
