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
      first_image_sub_(it_, "raw/first/image", queue_size_),
      second_image_sub_(it_, "raw/second/image", queue_size_),
      first_undistorter_ptr_(nullptr),
      second_undistorter_ptr_(nullptr),
      frame_counter_(0) {
  // set parameters from ros
  nh_private_.param("input_camera_info_from_ros_params",
                    input_camera_info_from_ros_params_,
                    kDefaultInputCameraInfoFromROSParams);

  nh_private_.param("rename_radtan_plumb_bob", rename_radtan_plumb_bob_,
                    kDefaultRenameRadtanPlumbBob);

  bool invert_T;
  nh_private_.param("invert_T", invert_T, kDefaultInvertT);

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
  nh_private_.param("first_output_frame", first_output_frame_,
                    kDefaultFirstOutputFrame);
  if (first_output_frame_.empty()) {
    ROS_ERROR("First output frame cannot be blank, setting to default");
    first_output_frame_ = kDefaultFirstOutputFrame;
  }
  nh_private_.param("second_output_frame", second_output_frame_,
                    kDefaultSecondOutputFrame);
  if (second_output_frame_.empty()) {
    ROS_ERROR("Second output frame cannot be blank, setting to default");
    second_output_frame_ = kDefaultSecondOutputFrame;
  }

  nh_private_.param("rename_input_frame", rename_input_frame_,
                    kDefaultRenameInputFrame);
  nh_private_.param("first_input_frame", first_input_frame_,
                    kDefaultFirstInputFrame);
  if (first_input_frame_.empty()) {
    ROS_ERROR("First input frame cannot be blank, setting to default");
    first_input_frame_ = kDefaultFirstInputFrame;
  }
  nh_private_.param("second_input_frame", second_input_frame_,
                    kDefaultSecondInputFrame);
  if (second_input_frame_.empty()) {
    ROS_ERROR("Second input frame cannot be blank, setting to default");
    second_input_frame_ = kDefaultSecondInputFrame;
  }

  // setup publishers
  first_camera_info_output_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
      "rect/first/camera_info", queue_size_);
  second_camera_info_output_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
      "rect/second/camera_info", queue_size_);
  first_image_pub_ = it_.advertise("rect/first/image", queue_size_);
  second_image_pub_ = it_.advertise("rect/second/image", queue_size_);

  // setup subscribers (must be done last as it appears image filters allow a
  // subscriber to be called as soon as it is created, even if this constructor
  // is not finished)
  if (input_camera_info_from_ros_params_) {
    std::string first_camera_namespace, second_camera_namespace;
    nh_private_.param("first_camera_namespace", first_camera_namespace,
                      kDefaultFirstCameraNamespace);
    nh_private_.param("second_camera_namespace", second_camera_namespace,
                      kDefaultSecondCameraNamespace);
    if (!stereo_camera_parameters_ptr_->setInputCameraParameters(
            nh_private_, first_camera_namespace, CameraSide::FIRST, invert_T) ||
        !stereo_camera_parameters_ptr_->setInputCameraParameters(
            nh_private_, second_camera_namespace, CameraSide::SECOND,
            invert_T)) {
      ROS_FATAL("Loading of input camera parameters failed, exiting");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }

    first_camera_info_input_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
        "raw/first/camera_info", queue_size_);
    second_camera_info_input_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
        "raw/second/camera_info", queue_size_);

    image_sync_ptr_ =
        std::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(
            ImageSyncPolicy(queue_size_), first_image_sub_, second_image_sub_);
    image_sync_ptr_->registerCallback(
        boost::bind(&StereoUndistort::imagesCallback, this, _1, _2));

  } else {
    first_camera_info_sub_ptr_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(
            nh_, "raw/first/camera_info", queue_size_);
    second_camera_info_sub_ptr_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(
            nh_, "raw/second/camera_info", queue_size_);

    camera_sync_ptr_ =
        std::make_shared<message_filters::Synchronizer<CameraSyncPolicy>>(
            CameraSyncPolicy(queue_size_), first_image_sub_, second_image_sub_,
            *first_camera_info_sub_ptr_, *second_camera_info_sub_ptr_);
    camera_sync_ptr_->registerCallback(
        boost::bind(&StereoUndistort::camerasCallback, this, _1, _2, _3, _4));
  }
}

void StereoUndistort::updateUndistorter(const CameraSide& side) {
  std::shared_ptr<Undistorter>* undistorter_ptr_ptr;
  CameraParametersPair camera_parameters_pair;

  if (side == CameraSide::FIRST) {
    undistorter_ptr_ptr = &first_undistorter_ptr_;
    camera_parameters_pair = stereo_camera_parameters_ptr_->getFirst();
  } else {
    undistorter_ptr_ptr = &second_undistorter_ptr_;
    camera_parameters_pair = stereo_camera_parameters_ptr_->getSecond();
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

  if (side == CameraSide::FIRST) {
    image_out_ptr->header.frame_id = first_output_frame_;

    first_undistorter_ptr_->undistortImage(image_in_ptr->image,
                                           &(image_out_ptr->image));
    first_image_pub_.publish(*(image_out_ptr->toImageMsg()));

    if (publish_tf_) {
      Eigen::Matrix4d T =
          stereo_camera_parameters_ptr_->getFirst().getOutputPtr()->T();

      tf::Matrix3x3 R_ros;
      tf::Vector3 p_ros;
      tf::matrixEigenToTF(T.topLeftCorner<3, 3>(), R_ros);
      tf::vectorEigenToTF(T.topRightCorner<3, 1>(), p_ros);
      tf::Transform(R_ros, p_ros);

      std::string frame = image_in_ptr->header.frame_id;
      if (rename_input_frame_) {
        frame = first_input_frame_;
      }
      if (frame.empty()) {
        ROS_ERROR_ONCE("Image frame name is blank, cannot construct tf");
      } else {
        br_.sendTransform(tf::StampedTransform(tf::Transform(R_ros, p_ros),
                                               image_out_ptr->header.stamp,
                                               frame, first_output_frame_));
      }
    }
  } else {
    image_out_ptr->header.frame_id = second_output_frame_;

    second_undistorter_ptr_->undistortImage(image_in_ptr->image,
                                            &(image_out_ptr->image));
    second_image_pub_.publish(*(image_out_ptr->toImageMsg()));

    if (publish_tf_) {
      Eigen::Matrix4d T =
          stereo_camera_parameters_ptr_->getSecond().getOutputPtr()->T();

      tf::Matrix3x3 R_ros;
      tf::Vector3 p_ros;
      tf::matrixEigenToTF(T.topLeftCorner<3, 3>(), R_ros);
      tf::vectorEigenToTF(T.topRightCorner<3, 1>(), p_ros);
      tf::Transform(R_ros, p_ros);

      std::string frame = image_in_ptr->header.frame_id;
      if (rename_input_frame_) {
        frame = second_input_frame_;
      }
      if (frame.empty()) {
        ROS_ERROR_ONCE("Image frame name is blank, cannot construct tf");
      } else {
        br_.sendTransform(tf::StampedTransform(tf::Transform(R_ros, p_ros),
                                               image_out_ptr->header.stamp,
                                               frame, second_output_frame_));
      }
    }
  }
}

void StereoUndistort::imagesCallback(
    const sensor_msgs::ImageConstPtr& first_image_msg_in,
    const sensor_msgs::ImageConstPtr& second_image_msg_in) {
  if (!stereo_camera_parameters_ptr_->valid()) {
    ROS_ERROR("Camera parameters invalid, undistortion failed");
    return;
  }

  if (++frame_counter_ < process_every_nth_frame_) {
    return;
  }
  frame_counter_ = 0;

  processAndSendImage(first_image_msg_in, CameraSide::FIRST);
  processAndSendImage(second_image_msg_in, CameraSide::SECOND);

  if (input_camera_info_from_ros_params_) {
    std_msgs::Header header = first_image_msg_in->header;
    if (rename_input_frame_) {
      header.frame_id = first_input_frame_;
    }
    sendCameraInfo(header, CameraSide::FIRST, CameraIO::INPUT);

    header = second_image_msg_in->header;
    if (rename_input_frame_) {
      header.frame_id = second_input_frame_;
    }
    sendCameraInfo(header, CameraSide::SECOND, CameraIO::INPUT);
  }

  std_msgs::Header header = first_image_msg_in->header;
  header.frame_id = first_output_frame_;
  sendCameraInfo(header, CameraSide::FIRST, CameraIO::OUTPUT);

  header = second_image_msg_in->header;
  header.frame_id = second_output_frame_;
  sendCameraInfo(header, CameraSide::SECOND, CameraIO::OUTPUT);
}

void StereoUndistort::camerasCallback(
    const sensor_msgs::ImageConstPtr& first_image_msg_in,
    const sensor_msgs::ImageConstPtr& second_image_msg_in,
    const sensor_msgs::CameraInfoConstPtr& first_camera_info_msg_in,
    const sensor_msgs::CameraInfoConstPtr& second_camera_info_msg_in) {
  if (!stereo_camera_parameters_ptr_->setInputCameraParameters(
          *first_camera_info_msg_in, CameraSide::FIRST) ||
      !stereo_camera_parameters_ptr_->setInputCameraParameters(
          *second_camera_info_msg_in, CameraSide::SECOND)) {
    ROS_ERROR("Setting camera info failed, dropping frame");
    return;
  }

  imagesCallback(first_image_msg_in, second_image_msg_in);
}
}  // namespace image_undistort
