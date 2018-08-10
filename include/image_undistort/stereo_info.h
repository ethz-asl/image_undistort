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
constexpr int kQueueSize = 10;
// true to load input cam_info from ros parameters, false to get it from a
// cam_info topic
constexpr bool kDefaultInputCameraInfoFromROSParams = true;
// namespace to use when loading first camera parameters from ros params
const std::string kDefaultFirstCameraNamespace = "first_camera";
// namespace to use when loading second camera parameters from ros params
const std::string kDefaultSecondCameraNamespace = "second_camera";
// the output focal length will be multiplied by this value. This has the effect
// of resizing the image by this scale factor.
constexpr double kDefaultScale = 1.0;
// if radtan distortion should be called radtan (ASL standard) or plumb_bob (ros
// standard)
constexpr bool kDefaultRenameRadtanPlumbBob = false;
// if the read in transforms should be inverted
constexpr bool kDefaultInvertT = false;

class StereoInfo {
 public:
  StereoInfo(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void firstImageCallback(const sensor_msgs::ImageConstPtr& image_msg);

  void secondImageCallback(const sensor_msgs::ImageConstPtr& image_msg);

  void firstCameraInfoCallback(
      const sensor_msgs::CameraInfoConstPtr& camera_info);

  void secondCameraInfoCallback(
      const sensor_msgs::CameraInfoConstPtr& camera_info);

 private:
  void sendCameraInfo(const std_msgs::Header& header, const CameraSide& side,
                      const CameraIO& io);

  // nodes
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  image_transport::ImageTransport it_;

  // subscribers
  image_transport::Subscriber first_image_sub_;
  image_transport::Subscriber second_image_sub_;
  ros::Subscriber first_camera_info_sub_;
  ros::Subscriber second_camera_info_sub_;

  // publishers
  ros::Publisher first_camera_info_input_pub_;
  ros::Publisher second_camera_info_input_pub_;
  ros::Publisher first_camera_info_output_pub_;
  ros::Publisher second_camera_info_output_pub_;

  std::shared_ptr<StereoCameraParameters> stereo_camera_parameters_ptr_;

  int queue_size_;
  bool rename_radtan_plumb_bob_;
};
}

#endif
