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

  bool invert_T;
  nh_private_.param("invert_T", invert_T, kDefaultInvertT);

  double scale;
  nh_private_.param("scale", scale, kDefaultScale);

  

  stereo_camera_parameters_ptr_ =
      std::make_shared<StereoCameraParameters>(scale);

  // setup subscribers
  if (input_camera_info_from_ros_params) {
    std::string first_camera_namespace, second_camera_namespace;
    nh_private_.param("first_camera_namespace", first_camera_namespace,
                      kDefaultFirstCameraNamespace);
    nh_private_.param("second_camera_namespace", second_camera_namespace,
                      kDefaultSecondCameraNamespace);
    if (!stereo_camera_parameters_ptr_->setInputCameraParameters(
            nh_private_, first_camera_namespace, CameraSide::FIRST, invert_T) ||
        !stereo_camera_parameters_ptr_->setInputCameraParameters(
            nh_private_, second_camera_namespace, CameraSide::SECOND, invert_T)) {
      ROS_FATAL("Loading of input camera parameters failed, exiting");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }
    first_image_sub_ = it_.subscribe("raw/first/image", queue_size_,
                                     &StereoInfo::firstImageCallback, this);
    second_image_sub_ = it_.subscribe("raw/second/image", queue_size_,
                                      &StereoInfo::secondImageCallback, this);

    first_camera_info_input_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
        "raw/first/camera_info", queue_size_);
    second_camera_info_input_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
        "raw/second/camera_info", queue_size_);
  } else {
    first_camera_info_sub_ =
        nh_.subscribe("raw/first/camera_info", queue_size_,
                      &StereoInfo::firstCameraInfoCallback, this);
    second_camera_info_sub_ =
        nh_.subscribe("raw/second/camera_info", queue_size_,
                      &StereoInfo::secondCameraInfoCallback, this);
  }

  // setup publishers
  first_camera_info_output_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
      "rect/first/camera_info", queue_size_);
  second_camera_info_output_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
      "rect/second/camera_info", queue_size_);
}

void StereoInfo::firstImageCallback(
    const sensor_msgs::ImageConstPtr& image_msg) {
  sendCameraInfo(image_msg->header, CameraSide::FIRST, CameraIO::INPUT);
  sendCameraInfo(image_msg->header, CameraSide::FIRST, CameraIO::OUTPUT);
}

void StereoInfo::secondImageCallback(
    const sensor_msgs::ImageConstPtr& image_msg) {
  sendCameraInfo(image_msg->header, CameraSide::SECOND, CameraIO::INPUT);
  sendCameraInfo(image_msg->header, CameraSide::SECOND, CameraIO::OUTPUT);
}

void StereoInfo::firstCameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr& camera_info) {
  if (!stereo_camera_parameters_ptr_->setInputCameraParameters(
          *camera_info, CameraSide::FIRST)) {
    ROS_ERROR("Setting first camera parameters from camera info failed");
  } else {
    sendCameraInfo(camera_info->header, CameraSide::FIRST, CameraIO::OUTPUT);
  }
}

void StereoInfo::secondCameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr& camera_info) {
  if (!stereo_camera_parameters_ptr_->setInputCameraParameters(
          *camera_info, CameraSide::SECOND)) {
    ROS_ERROR("Setting second camera parameters from camera info failed");
  } else {
    sendCameraInfo(camera_info->header, CameraSide::SECOND, CameraIO::OUTPUT);
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

  if (side == CameraSide::FIRST) {
    if (io == CameraIO::INPUT) {
      first_camera_info_input_pub_.publish(camera_info);
    } else {
      first_camera_info_output_pub_.publish(camera_info);
    }
  } else {
    if (io == CameraIO::INPUT) {
      second_camera_info_input_pub_.publish(camera_info);
    } else {
      second_camera_info_output_pub_.publish(camera_info);
    }
  }
}
}