#ifndef IMAGE_UNDISTORT_STEREO_UNDISTORTER_H
#define IMAGE_UNDISTORT_STEREO_UNDISTORTER_H

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>

#include "image_undistort/camera_parameters.h"
#include "image_undistort/undistorter.h"

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
// downsamples output rate if <= 1, every frame is processed.
constexpr int kDefaultProcessEveryNthFrame = 1;
// converts the output image to the given format, set to the empty string "" to
// copy input type. Consult cv_bridge documentation for valid strings
const std::string kDefaultOutputImageType = "";
// if the output camera source is set to "auto_generated", the output focal
// length will be multiplied by this value. This has the effect of
// resizing the image by this scale factor.
constexpr double kDefaultScale = 1.0;
// if a tf between the input and output frame should be created
constexpr bool kDefaultPublishTF = true;
// name of output image frame
const std::string kDefaultOutputFrame = "left_camera_rect";
// if radtan distortion should be called radtan (ASL standard) or plumb_bob (ros
// standard)
constexpr bool kDefaultRenameRadtanPlumbBob = false;

class StereoUndistort {
 public:
  StereoUndistort(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void imagesCallback(const sensor_msgs::ImageConstPtr& left_image_msg_in,
                      const sensor_msgs::ImageConstPtr& right_image_msg_in);

  void camerasCallback(
      const sensor_msgs::ImageConstPtr& left_image_msg_in,
      const sensor_msgs::ImageConstPtr& right_image_msg_in,
      const sensor_msgs::CameraInfoConstPtr& left_camera_info_msg_in,
      const sensor_msgs::CameraInfoConstPtr& right_camera_info_msg_in);

 private:
  int getQueueSize();

  void updateUndistorter(bool left);

  void sendCameraInfo(const std_msgs::Header& header, const bool left,
                      const bool input);

  void processAndSendImage(const sensor_msgs::ImageConstPtr& image_msg_in,
                           const bool left);

  // tf broadcaster
  tf::TransformBroadcaster br_;

  // nodes
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  image_transport::ImageTransport it_;

  // subscribers
  image_transport::SubscriberFilter left_image_sub_;
  image_transport::SubscriberFilter right_image_sub_;
  // these use pointers as they are not needed when loading from ros params
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>>
      left_camera_info_sub_ptr_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>>
      right_camera_info_sub_ptr_;

  // publishers
  image_transport::Publisher left_image_pub_;
  image_transport::Publisher right_image_pub_;
  ros::Publisher left_camera_info_input_pub_;
  ros::Publisher right_camera_info_input_pub_;
  ros::Publisher left_camera_info_output_pub_;
  ros::Publisher right_camera_info_output_pub_;

  // filters
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      ImageSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy>>
      image_sync_ptr_;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
      sensor_msgs::CameraInfo>
      CameraSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<CameraSyncPolicy>>
      camera_sync_ptr_;

  // camera parameters
  std::shared_ptr<StereoCameraParameters> stereo_camera_parameters_ptr_;

  // undistorters
  std::shared_ptr<Undistorter> left_undistorter_ptr_;
  std::shared_ptr<Undistorter> right_undistorter_ptr_;

  // other variables
  bool input_camera_info_from_ros_params_;
  int queue_size_;
  int process_every_nth_frame_;
  std::string output_image_type_;
  bool publish_tf_;
  std::string output_frame_;
  bool rename_radtan_plumb_bob_;
  int frame_counter_;
};
}

#endif