#include "image_undistort/stereo_info.h"
#include "image_undistort/camera_parameters.h"

namespace image_undistort {

StereoInfo::StereoInfo(const ros::NodeHandle& nh,
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

  nh_private_.param("rename_radtan_plumb_bob", rename_radtan_plumb_bob_,
                    kDefaultRenameRadtanPlumbBob);

  double scale;
  nh_private_.param("scale", scale, kDefaultScale);

  stereo_camera_parameters_ptr_ =
      std::make_shared<StereoCameraParameters>(scale);

  // setup subscribers
  if (input_camera_info_from_ros_params) {
    std::string left_camera_namespace, right_camera_namespace;
    nh_private_.param("left_camera_namespace", left_camera_namespace,
                      kDefaultLeftCameraNamespace);
    nh_private_.param("right_camera_namespace", right_camera_namespace,
                      kDefaultRightCameraNamespace);
    if (!stereo_camera_parameters_ptr_->setInputCameraParameters(
            nh_private_, left_camera_namespace, CameraSide::LEFT) ||
        !stereo_camera_parameters_ptr_->setInputCameraParameters(
            nh_private_, right_camera_namespace, CameraSide::RIGHT)) {
      ROS_FATAL("Loading of input camera parameters failed, exiting");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }
    left_image_sub_ = it_.subscribe("raw/left/image", queue_size_,
                                    &StereoInfo::leftImageCallback, this);
    right_image_sub_ = it_.subscribe("raw/right/image", queue_size_,
                                     &StereoInfo::rightImageCallback, this);

    left_camera_info_input_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
        "raw/left/camera_info", queue_size_);
    right_camera_info_input_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
        "raw/right/camera_info", queue_size_);
  } else {
    left_camera_info_sub_ =
        nh_.subscribe("raw/left/camera_info", queue_size_,
                      &StereoInfo::leftCameraInfoCallback, this);
    right_camera_info_sub_ =
        nh_.subscribe("raw/right/camera_info", queue_size_,
                      &StereoInfo::rightCameraInfoCallback, this);
  }

  // setup publishers
  left_camera_info_output_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
      "rect/left/camera_info", queue_size_);
  right_camera_info_output_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
      "rect/right/camera_info", queue_size_);
}

void StereoInfo::leftImageCallback(
    const sensor_msgs::ImageConstPtr& image_msg) {
  sendCameraInfo(image_msg->header, CameraSide::LEFT, CameraIO::INPUT);
  sendCameraInfo(image_msg->header, CameraSide::LEFT, CameraIO::OUTPUT);
}

void StereoInfo::rightImageCallback(
    const sensor_msgs::ImageConstPtr& image_msg) {
  sendCameraInfo(image_msg->header, CameraSide::RIGHT, CameraIO::INPUT);
  sendCameraInfo(image_msg->header, CameraSide::RIGHT, CameraIO::OUTPUT);
}

void StereoInfo::leftCameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr& camera_info) {
  if (!stereo_camera_parameters_ptr_->setInputCameraParameters(
          *camera_info, CameraSide::LEFT)) {
    ROS_ERROR("Setting left camera parameters from camera info failed");
  } else {
    sendCameraInfo(camera_info->header, CameraSide::LEFT, CameraIO::OUTPUT);
  }
}

void StereoInfo::rightCameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr& camera_info) {
  if (!stereo_camera_parameters_ptr_->setInputCameraParameters(
          *camera_info, CameraSide::RIGHT)) {
    ROS_ERROR("Setting right camera parameters from camera info failed");
  } else {
    sendCameraInfo(camera_info->header, CameraSide::RIGHT, CameraIO::OUTPUT);
  }
}

void StereoInfo::sendCameraInfo(const std_msgs::Header& header,
                                const CameraSide& side, const CameraIO& io) {
  sensor_msgs::CameraInfo camera_info;
  camera_info.header = header;
  try {
    stereo_camera_parameters_ptr_->generateCameraInfoMessage(side, io,
                                                             &camera_info);
  } catch (std::runtime_error e) {
    ROS_ERROR("%s", e.what());
    return;
  }

  if (rename_radtan_plumb_bob_ && camera_info.distortion_model == "radtan") {
    camera_info.distortion_model = "plumb_bob";
  }

  if (side == CameraSide::LEFT) {
    if (io == CameraIO::INPUT) {
      left_camera_info_input_pub_.publish(camera_info);
    } else {
      left_camera_info_output_pub_.publish(camera_info);
    }
  } else {
    if (io == CameraIO::INPUT) {
      right_camera_info_input_pub_.publish(camera_info);
    } else {
      right_camera_info_output_pub_.publish(camera_info);
    }
  }
}
}