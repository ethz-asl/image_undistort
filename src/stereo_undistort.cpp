#include "image_undistort/stereo_undistort.h"
#include "image_undistort/camera_parameters.h"

namespace image_undistort {

// needed so that topic filters can be initalized in the constructor
int StereoUndistort::getQueueSize() const {
  int queue_size;
  nh_private_.param("queue_size", queue_size, kQueueSize);
  if (queue_size < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size = 1;
  }
  return queue_size;
}

StereoUndistort::StereoUndistort(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      it_(nh_),
      queue_size_(getQueueSize()),
      left_image_sub_(it_, "raw/left/image", queue_size_),
      right_image_sub_(it_, "raw/right/image", queue_size_),
      left_undistorter_ptr_(nullptr),
      right_undistorter_ptr_(nullptr),
      frame_counter_(0) {
  // set parameters from ros
  nh_private_.param("input_camera_info_from_ros_params",
                    input_camera_info_from_ros_params_,
                    kDefaultInputCameraInfoFromROSParams);

  nh_private_.param("rename_radtan_plumb_bob", rename_radtan_plumb_bob_,
                    kDefaultRenameRadtanPlumbBob);

  nh_private_.param("queue_size", queue_size_, kQueueSize);
  if (queue_size_ < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size_ = 1;
  }

  double scale;
  nh_private_.param("scale", scale, kDefaultScale);

  stereo_camera_parameters_ptr_ =
      std::make_shared<StereoCameraParameters>(scale);

  nh_private_.param("process_every_nth_frame", process_every_nth_frame_,
                    kDefaultProcessEveryNthFrame);
  nh_private_.param("output_image_type", output_image_type_,
                    kDefaultOutputImageType);
  // check output type string is correctly formatted
  if (!output_image_type_.empty()) {
    try {
      cv_bridge::getCvType(output_image_type_);
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR_STREAM(
          "cv_bridge error while setting output_image_type, output will match "
          "input type. "
          << e.what());
      output_image_type_ = "";
    }
  }

  nh_private_.param("publish_tf", publish_tf_, kDefaultPublishTF);
  nh_private_.param("output_frame", output_frame_, kDefaultOutputFrame);
  if (output_frame_.empty()) {
    ROS_ERROR("Output frame cannot be blank, setting to default");
    output_frame_ = kDefaultOutputFrame;
  }

  nh_private_.param("rename_input_frame", rename_input_frame_,
                    kDefaultRenameInputFrame);
  nh_private_.param("left_input_frame", left_input_frame_,
                    kDefaultLeftInputFrame);
  if (left_input_frame_.empty()) {
    ROS_ERROR("Left input frame cannot be blank, setting to default");
    left_input_frame_ = kDefaultLeftInputFrame;
  }
  nh_private_.param("right_input_frame", right_input_frame_,
                    kDefaultRightInputFrame);
  if (right_input_frame_.empty()) {
    ROS_ERROR("Right input frame cannot be blank, setting to default");
    right_input_frame_ = kDefaultRightInputFrame;
  }

  // setup publishers
  left_camera_info_output_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
      "rect/left/camera_info", queue_size_);
  right_camera_info_output_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
      "rect/right/camera_info", queue_size_);
  left_image_pub_ = it_.advertise("rect/left/image", queue_size_);
  right_image_pub_ = it_.advertise("rect/right/image", queue_size_);

  // setup subscribers (must be done last as it appears image filters allow a
  // subscriber to be called as soon as it is created, even if this constructor
  // is not finished)
  if (input_camera_info_from_ros_params_) {
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

    left_camera_info_input_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
        "raw/left/camera_info", queue_size_);
    right_camera_info_input_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
        "raw/right/camera_info", queue_size_);

    image_sync_ptr_ =
        std::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(
            ImageSyncPolicy(queue_size_), left_image_sub_, right_image_sub_);
    image_sync_ptr_->registerCallback(
        boost::bind(&StereoUndistort::imagesCallback, this, _1, _2));

  } else {
    left_camera_info_sub_ptr_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(
            nh_, "raw/left/camera_info", queue_size_);
    right_camera_info_sub_ptr_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(
            nh_, "raw/right/camera_info", queue_size_);

    camera_sync_ptr_ =
        std::make_shared<message_filters::Synchronizer<CameraSyncPolicy>>(
            CameraSyncPolicy(queue_size_), left_image_sub_, right_image_sub_,
            *left_camera_info_sub_ptr_, *right_camera_info_sub_ptr_);
    camera_sync_ptr_->registerCallback(
        boost::bind(&StereoUndistort::camerasCallback, this, _1, _2, _3, _4));
  }
}

void StereoUndistort::updateUndistorter(const CameraSide& side) {
  std::shared_ptr<Undistorter>* undistorter_ptr_ptr;
  CameraParametersPair camera_parameters_pair;

  if (side == CameraSide::LEFT) {
    undistorter_ptr_ptr = &left_undistorter_ptr_;
    camera_parameters_pair = stereo_camera_parameters_ptr_->getLeft();
  } else {
    undistorter_ptr_ptr = &right_undistorter_ptr_;
    camera_parameters_pair = stereo_camera_parameters_ptr_->getRight();
  }

  // if undistorter not built or built using old data update it
  if (!(*undistorter_ptr_ptr) ||
      ((*undistorter_ptr_ptr)->getCameraParametersPair() !=
       camera_parameters_pair)) {
    try {
      *undistorter_ptr_ptr =
          std::make_shared<Undistorter>(camera_parameters_pair);
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
      return;
    }
  }
}

void StereoUndistort::sendCameraInfo(const std_msgs::Header& header,
                                     const CameraSide& side,
                                     const CameraIO& io) {
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

void StereoUndistort::processAndSendImage(
    const sensor_msgs::ImageConstPtr& image_msg_in, const CameraSide& side) {
  cv_bridge::CvImageConstPtr image_in_ptr =
      cv_bridge::toCvShare(image_msg_in, output_image_type_);

  std::string encoding = image_in_ptr->encoding;
  if (encoding == "8UC1") {
    // ros does not recognize U8C1 and it will crash the disparity node
    encoding = "mono8";
  }
  cv_bridge::CvImagePtr image_out_ptr(
      new cv_bridge::CvImage(image_in_ptr->header, encoding));

  updateUndistorter(side);

  image_out_ptr->header.frame_id = output_frame_;

  if (side == CameraSide::LEFT) {
    left_undistorter_ptr_->undistortImage(image_in_ptr->image,
                                          &(image_out_ptr->image));
    left_image_pub_.publish(*(image_out_ptr->toImageMsg()));

    if (publish_tf_) {
      Eigen::Matrix4d T =
          stereo_camera_parameters_ptr_->getLeft()
              .getInputPtr()
              ->T()
              .inverse() *
          stereo_camera_parameters_ptr_->getRight().getOutputPtr()->T();

      tf::Matrix3x3 R_ros;
      tf::Vector3 p_ros;
      tf::matrixEigenToTF(T.topLeftCorner<3, 3>(), R_ros);
      tf::vectorEigenToTF(T.topRightCorner<3, 1>(), p_ros);
      tf::Transform(R_ros, p_ros);

      std::string frame = image_in_ptr->header.frame_id;
      if (rename_input_frame_) {
        frame = left_input_frame_;
      }
      if (frame.empty()) {
        ROS_ERROR_ONCE("Image frame name is blank, cannot construct tf");
      } else {
        br_.sendTransform(tf::StampedTransform(tf::Transform(R_ros, p_ros),
                                               image_out_ptr->header.stamp,
                                               frame, output_frame_));
      }
    }
  } else {
    right_undistorter_ptr_->undistortImage(image_in_ptr->image,
                                           &(image_out_ptr->image));
    right_image_pub_.publish(*(image_out_ptr->toImageMsg()));
  }
}

void StereoUndistort::imagesCallback(
    const sensor_msgs::ImageConstPtr& left_image_msg_in,
    const sensor_msgs::ImageConstPtr& right_image_msg_in) {
  if (!stereo_camera_parameters_ptr_->valid()) {
    ROS_ERROR("Camera parameters invalid, undistortion failed");
    return;
  }

  if (++frame_counter_ < process_every_nth_frame_) {
    return;
  }
  frame_counter_ = 0;

  processAndSendImage(left_image_msg_in, CameraSide::LEFT);
  processAndSendImage(right_image_msg_in, CameraSide::RIGHT);

  if (input_camera_info_from_ros_params_) {
    std_msgs::Header header = left_image_msg_in->header;
    if (rename_input_frame_) {
      header.frame_id = left_input_frame_;
    }
    sendCameraInfo(header, CameraSide::LEFT, CameraIO::INPUT);

    header = right_image_msg_in->header;
    if (rename_input_frame_) {
      header.frame_id = right_input_frame_;
    }
    sendCameraInfo(header, CameraSide::RIGHT, CameraIO::INPUT);
  }

  std_msgs::Header header = left_image_msg_in->header;
  header.frame_id = output_frame_;
  sendCameraInfo(header, CameraSide::LEFT, CameraIO::OUTPUT);

  header = right_image_msg_in->header;
  header.frame_id = output_frame_;
  sendCameraInfo(header, CameraSide::RIGHT, CameraIO::OUTPUT);
}

void StereoUndistort::camerasCallback(
    const sensor_msgs::ImageConstPtr& left_image_msg_in,
    const sensor_msgs::ImageConstPtr& right_image_msg_in,
    const sensor_msgs::CameraInfoConstPtr& left_camera_info_msg_in,
    const sensor_msgs::CameraInfoConstPtr& right_camera_info_msg_in) {
  if (!stereo_camera_parameters_ptr_->setInputCameraParameters(
          *left_camera_info_msg_in, CameraSide::LEFT) ||
      !stereo_camera_parameters_ptr_->setInputCameraParameters(
          *right_camera_info_msg_in, CameraSide::RIGHT)) {
    ROS_ERROR("Setting camera info failed, dropping frame");
    return;
  }

  imagesCallback(left_image_msg_in, right_image_msg_in);
}
}
