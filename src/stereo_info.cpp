#include "image_undistort/camera_parameters.h"
#include "image_undistort/image_undistort.h"
#include "image_undistort/undistorter.h"

ImageUndistort::ImageUndistort(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), it_(nh_) {
  // set parameters from ros
  bool input_camera_info_from_ros_params;
  nh_private_.param("input_camera_info_from_ros_params",
                    input_camera_info_from_ros_params,
                    kDefaultInputCameraInfoFromROSParams);

  nh_private_.param("queue_size", queue_size_, kQueueSize);
  if (queue_size_ < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size_ = 1;
  }

  double scale;
  nh_private_.param("scale", scale, kDefaultScale);

  stero_parameters_ptr_ = std::make_shared<StereoParameters>(scale);

  // setup subscribers
  if (input_camera_info_from_ros_params) {
    std::string left_camera_namespace, right_camera_namespace;
    nh_private_.param("left_camera_namespace", left_camera_namespace,
                      kDefaultLeftCameraNamespace);
    nh_private_.param("right_camera_namespace", right_camera_namespace,
                      kDefaultRightCameraNamespace);
    if (!stereo_parameters_ptr_->setInputCameraParameters(
            nh_private_, left_camera_namespace, true) ||
        !stereo_parameters_ptr_->setInputCameraParameters(
            nh_private_, right_camera_namespace, false)) {
      ROS_FATAL("Loading of input camera parameters failed, exiting");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }
    left_image_sub_ = it_.subscribe("left/image", queue_size_,
                                    &StereoInfo::leftImageCallback, this);
    right_image_sub_ = it_.subscribe("right/image", queue_size_,
                                     &StereoInfo::rightImageCallback, this);
  } else {
    left_camera_info_sub_ =
        nh_.subscribe("left/camera_info", queue_size_,
                      &StereoInfo::leftCameraInfoCallback, this);
    right_camera_info_sub_ =
        nh_.subscribe("right/camera_info", queue_size_,
                      &StereoInfo::rightCameraInfoCallback, this);
  }

  // setup publishers
  left_camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
      "output/left/camera_info_rectified", queue_size_);
  right_camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
      "output/right/camera_info_rectified", queue_size_);
}

void ImageUndistort::leftImageCallback(
    const sensor_msgs::ImageConstPtr& image_msg) {
    sendCameraInfo(image_msg->header, true);
}

void ImageUndistort::rightImageCallback(
    const sensor_msgs::ImageConstPtr& image_msg) {
  sendCameraInfo(image_msg->header, false);
}

void leftCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info){
  if(!stereo_parameters_ptr_->setInputCameraParameters(camera_info, true)){
    ROS_ERROR("Setting left camera parameters from camera info failed");
  } else{
    sendCameraInfo(camera_info.header, true);
  }
}

void rightCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info){
  if(!stereo_parameters_ptr_->setInputCameraParameters(camera_info, false)){
    ROS_ERROR("Setting right camera parameters from camera info failed");
  } else{
    sendCameraInfo(camera_info.header, false);
  }
}

void sendCameraInfo(const std_msgs::Header& header,
                    const bool left) {
  sensor_msgs::CameraInfo camera_info;
  camera_info.header = header;
  camera_parameters_pair_ptr_->generateOutputCameraInfoMessage(left,
                                                               &camera_info);
  if (left) {
    left_camera_info_pub_.publish(camera_info);
  } else {
    right_camera_info_pub_.publish(camera_info);
  }
}
