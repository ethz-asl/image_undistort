#include "image_undistort/image_undistort.h"
#include "image_undistort/camera_parameters.h"
#include "image_undistort/undistorter.h"

namespace image_undistort {

ImageUndistort::ImageUndistort(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      it_(nh_),
      undistorter_ptr_(nullptr),
      frame_counter_(0) {
  // set parameters from ros
  bool input_camera_info_from_ros_params;
  nh_private_.param("input_camera_info_from_ros_params",
                    input_camera_info_from_ros_params,
                    kDefaultInputCameraInfoFromROSParams);

  nh_private_.param("rename_radtan_plumb_bob", rename_radtan_plumb_bob_,
                    kDefaultRenameRadtanPlumbBob);

  std::string output_camera_info_source_in;
  nh_private_.param("output_camera_info_source", output_camera_info_source_in,
                    kDefaultOutputCameraInfoSource);
  if (output_camera_info_source_in == "auto_generated") {
    output_camera_info_source_ = OutputInfoSource::AUTO_GENERATED;
  } else if (output_camera_info_source_in == "match_input") {
    output_camera_info_source_ = OutputInfoSource::MATCH_INPUT;
  } else if (output_camera_info_source_in == "ros_params") {
    output_camera_info_source_ = OutputInfoSource::ROS_PARAMS;
  } else if (output_camera_info_source_in == "camera_info") {
    output_camera_info_source_ = OutputInfoSource::CAMERA_INFO;
  } else {
    ROS_ERROR(
        "Invalid camera source given, valid options are auto_generated, "
        "match_input, ros_params and camera_info. Defaulting to "
        "auto_generated");
    output_camera_info_source_ = OutputInfoSource::AUTO_GENERATED;
  }

  nh_private_.param("queue_size", queue_size_, kImageQueueSize);
  if (queue_size_ < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size_ = 1;
  }
  nh_private_.param("process_image", process_image_, kDefaultProcessImage);
  if (!process_image_ && !input_camera_info_from_ros_params) {
    ROS_FATAL(
        "Settings specify no image processing and not to generate camera info "
        "from file. This leaves nothing for the node to do, exiting");
    ros::shutdown();
    exit(EXIT_SUCCESS);
  }
  nh_private_.param("scale", scale_, kDefaultScale);

  bool undistort_image;
  nh_private_.param("undistort_image", undistort_image, kDefaultUndistortImage);
  DistortionProcessing distortion_processing;
  if (undistort_image) {
    distortion_processing = DistortionProcessing::UNDISTORT;
  } else {
    distortion_processing = DistortionProcessing::PRESERVE;
  }
  camera_parameters_pair_ptr_ =
      std::make_shared<CameraParametersPair>(distortion_processing);

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
  nh_private_.param("input_frame", input_frame_, kDefaultInputFrame);
  if (input_frame_.empty()) {
    ROS_ERROR("Input frame cannot be blank, setting to default");
    input_frame_ = kDefaultInputFrame;
  }

  // setup subscribers
  std::string input_camera_namespace;
  if (input_camera_info_from_ros_params) {
    nh_private_.param("input_camera_namespace", input_camera_namespace,
                      kDefaultInputCameraNamespace);
    if (!camera_parameters_pair_ptr_->setCameraParameters(
            nh_private_, input_camera_namespace, CameraIO::INPUT)) {
      ROS_FATAL("Loading of input camera parameters failed, exiting");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }
    image_sub_ = it_.subscribe("input/image", queue_size_,
                               &ImageUndistort::imageCallback, this);
  } else {
    camera_sub_ = it_.subscribeCamera("input/image", queue_size_,
                                      &ImageUndistort::cameraCallback, this);
  }

  // setup publishers
  if (process_image_) {
    bool pub_camera_info_output = true;
    if (output_camera_info_source_ == OutputInfoSource::ROS_PARAMS) {
      std::string output_camera_namespace;
      nh_private_.param("output_camera_namespace", output_camera_namespace,
                        kDefaultOutputCameraNamespace);
      if (!camera_parameters_pair_ptr_->setCameraParameters(
              nh_private_, output_camera_namespace, CameraIO::OUTPUT)) {
        ROS_FATAL("Loading of output camera parameters failed, exiting");
        ros::shutdown();
        exit(EXIT_FAILURE);
      }
    } else if (output_camera_info_source_ == OutputInfoSource::MATCH_INPUT) {
      camera_parameters_pair_ptr_->setOutputFromInput(scale_);
    } else if (output_camera_info_source_ == OutputInfoSource::AUTO_GENERATED) {
      camera_parameters_pair_ptr_->setOptimalOutputCameraParameters(scale_);
    } else {
      camera_info_sub_ =
          nh_.subscribe("output/camera_info", queue_size_,
                        &ImageUndistort::cameraInfoCallback, this);
      pub_camera_info_output = false;
    }

    if (pub_camera_info_output) {
      camera_pub_ = it_.advertiseCamera("output/image", queue_size_);
    } else {
      image_pub_ = it_.advertise("output/image", queue_size_);
    }
  } else {
    camera_parameters_pair_ptr_->setOutputFromInput(scale_);

    camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(
        "output/camera_info", queue_size_);
  }
}

void ImageUndistort::imageCallback(
    const sensor_msgs::ImageConstPtr& image_msg_in) {
  if (++frame_counter_ < process_every_nth_frame_) {
    return;
  }
  frame_counter_ = 0;

  if (!process_image_) {
    sensor_msgs::CameraInfo camera_info;
    camera_info.header = image_msg_in->header;
    if (rename_input_frame_) {
      camera_info.header.frame_id = input_frame_;
    }
    camera_parameters_pair_ptr_->generateCameraInfoMessage(CameraIO::OUTPUT,
                                                           &camera_info);
    if (rename_radtan_plumb_bob_ && camera_info.distortion_model == "radtan") {
      camera_info.distortion_model = "plumb_bob";
    }
    camera_info_pub_.publish(camera_info);
    return;
  }
  cv_bridge::CvImageConstPtr image_in_ptr =
      cv_bridge::toCvShare(image_msg_in, output_image_type_);

  std::string encoding = image_in_ptr->encoding;
  if (encoding == "8UC1") {
    // ros does not recognize U8C1 and using it will crash anything that does a
    // color conversion
    encoding = "mono8";
  }
  cv_bridge::CvImagePtr image_out_ptr(
      new cv_bridge::CvImage(image_in_ptr->header, encoding));

  // if undistorter not built or built using old data update it
  if (!undistorter_ptr_ || (undistorter_ptr_->getCameraParametersPair() !=
                            *camera_parameters_pair_ptr_)) {
    try {
      undistorter_ptr_ =
          std::make_shared<Undistorter>(*camera_parameters_pair_ptr_);
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
      return;
    }
  }

  undistorter_ptr_->undistortImage(image_in_ptr->image,
                                   &(image_out_ptr->image));

  image_out_ptr->header.frame_id = output_frame_;


  if (publish_tf_) {
    Eigen::Matrix4d T =
        camera_parameters_pair_ptr_->getInputPtr()->T().inverse() *
        camera_parameters_pair_ptr_->getOutputPtr()->T();

    tf::Matrix3x3 R_ros;
    tf::Vector3 p_ros;
    tf::matrixEigenToTF(T.topLeftCorner<3, 3>(), R_ros);
    tf::vectorEigenToTF(T.topRightCorner<3, 1>(), p_ros);
    tf::Transform(R_ros, p_ros);

    std::string frame = image_in_ptr->header.frame_id;
    if (rename_input_frame_) {
      frame = input_frame_;
    }
    if (frame.empty()) {
      ROS_ERROR_ONCE("Image frame name is blank, cannot construct tf");
    } else {
      br_.sendTransform(tf::StampedTransform(tf::Transform(R_ros, p_ros),
                                             image_out_ptr->header.stamp, frame,
                                             output_frame_));
    }
  }

  // if camera info was just read in from a topic don't republish it
  if (output_camera_info_source_ == OutputInfoSource::CAMERA_INFO) {
    image_pub_.publish(*(image_out_ptr->toImageMsg()));
  } else {
    sensor_msgs::CameraInfo camera_info;
    camera_info.header = image_out_ptr->header;
    if (rename_input_frame_) {
      camera_info.header.frame_id = input_frame_;
    }
    camera_parameters_pair_ptr_->generateCameraInfoMessage(CameraIO::OUTPUT,
                                                           &camera_info);
    if (rename_radtan_plumb_bob_ && camera_info.distortion_model == "radtan") {
      camera_info.distortion_model = "plumb_bob";
    }
    camera_pub_.publish(*(image_out_ptr->toImageMsg()), camera_info);
  }
}

void ImageUndistort::cameraCallback(
    const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& camera_info) {
  camera_parameters_pair_ptr_->setCameraParameters(*camera_info,
                                                   CameraIO::INPUT);
  if (output_camera_info_source_ == OutputInfoSource::MATCH_INPUT) {
    camera_parameters_pair_ptr_->setOutputFromInput(scale_);
  } else if (output_camera_info_source_ == OutputInfoSource::AUTO_GENERATED) {
    camera_parameters_pair_ptr_->setOptimalOutputCameraParameters(scale_);
  }

  imageCallback(image_msg);
}

void ImageUndistort::cameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr& camera_info) {
  if (!camera_parameters_pair_ptr_->setCameraParameters(*camera_info,
                                                        CameraIO::OUTPUT)) {
    ROS_ERROR("Setting output camera from ros message failed");
  }
}
}
