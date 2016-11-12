#include <image_undistort/camera_parameters.h>
#include <image_undistort/image_undistort.h>
#include <image_undistort/undistorter.h>

ImageUndistort::ImageUndistort(const ros::NodeHandle& nh,
                               const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      it_(nh_),
      undistorter_ptr_(nullptr),
      frame_counter_(0) {
  // set parameters from ros
  bool input_camera_info_from_yaml;
  private_nh_.param("input_camera_info_from_yaml", input_camera_info_from_yaml,
                    kDefaultInputCameraInfoFromYaml);
  private_nh_.param("output_camera_info_from_yaml",
                    output_camera_info_from_yaml_,
                    kDefaultOutputCameraInfoFromYaml);
  private_nh_.param("queue_size", queue_size_, kQueueSize);
  if (queue_size_ < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size_ = 1;
  }
  private_nh_.param("process_image", process_image_, kDefaultProcessImage);
  if (!process_image_ && !input_camera_info_from_yaml) {
    ROS_FATAL(
        "Settings specify no image processing and not to generate camera info "
        "from file. This leaves nothing for the node to do, exiting");
    ros::shutdown();
    exit(EXIT_SUCCESS);
  }

  bool undistort_image;
  private_nh_.param("undistort_image", undistort_image, kDefaultUndistortImage);
  camera_parameters_pair_ptr_ =
      std::make_shared<CameraParametersPair>(undistort_image);

  private_nh_.param("process_every_nth_frame", process_every_nth_frame_,
                    kDefaultProcessEveryNthFrame);
  private_nh_.param("output_image_type", output_image_type_,
                    kDefaultOutputImageType);
  // check output type string is correctly formatted
  if (!output_image_type_.empty()) {
    try {
      cv_bridge::getCvType(output_image_type_);
    } catch (const cv_brdige::exception& e) {
      ROS_ERROR_STREAM(
          "cv_bridge error while setting output_image_type, output will match "
          "input type. "
          << e.what());
      output_image_type_ = "";
    }
  }

  // setup subscribers
  if (input_camera_info_from_yaml) {
    private_nh_.param("input_camera_namespace", input_camera_namespace,
                      kDefaultInputCameraNamespace);
    if (!camera_parameters_pair_ptr_->setCameraParameters(
            private_nh_, input_camera_namespace, true)) {
      ROS_FATAL("Loading of input camera parameters failed, exiting");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }
    image_sub_ = it_.subscribe(image_topic, queue_size_,
                               &ImageUndistort::imageCallback, this);
  } else {
    camera_sub_ = it_.subscribeCamera("", queue_size_,
                                      &ImageUndistort::cameraCallback, this);
  }

  // setup publishers
  if (process_image_) {
    if (output_camera_info_from_yaml_) {
      std::string output_camera_namespace;
      private_nh_.param("output_camera_namespace", output_camera_namespace,
                        kDefaultOutputCameraNamespace);
      if (!camera_parameters_pair_ptr_->setCameraParameters(
              private_nh_, output_camera_namespace, false)) {
        ROS_FATAL("Loading of output camera parameters failed, exiting");
        ros::shutdown();
        exit(EXIT_FAILURE);
      }
    }
    camera_pub_ = it_.advertiseCamera("output", queue_size_);
  } else {
    // no processing so load input camera to output camera as well
    if (!camera_parameters_pair_ptr_->setCameraParameters(
            private_nh_, input_camera_namespace, false)) {
      ROS_FATAL("Loading of input camera parameters failed, exiting");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }

    camera_info_pub_ =
        nh_.advertise<sensor_msgs::CameraInfo>("cam_info", queue_size_);
  }
}

void ImageUndistort::imageCallback(
    const sensor_msgs::ImageConstPtr& image_msg_in) {
  if (++frame_counter_ < process_every_nth_frame_) {
    return;
  }
  frame_counter_ = 0;

  if (!process_image_) {
    sensor_msgs::CameraInfo* camera_info;
    camera_info.header.stamp = image_msg_in->header.stamp;
    camera_info.header.frame_id = image_msg_in->header.frame_id;
    camera_parameters_pair_ptr_->generateOutputCameraInfoMessage(&camera_info);
    camera_info_pub_.publish(camera_info);
  }
  cv_bridge::CvImageConstPtr image_in_ptr =
      cv_bridge::toCvShare(image_msg_in, output_image_type_);
  cv_bridge::CvImagePtr image_out_ptr(
      new cv_bridge::CvImage(image_in_ptr->header, image_in_ptr->encoding));

  // if undistorter not built or built using old data update it
  if (!undistorter_ptr || (undistorter_ptr->getCameraParametersPair !=
                           *camera_parameters_pair_ptr_)) {
    try {
      undistorter_ptr_ =
          std::make_shared<Undistorter>(*camera_parameters_pair_ptr_);
    } catch (std::runtime_error e) {
      ROS_ERROR(e.what());
      return;
    }
  }

  undistorter_ptr_->undistortImage(image_in_ptr->image,
                                   &(image_out_ptr->image));

  camera_info_out_.header.stamp = image_out_ptr->header.stamp;
  camera_info_out_.header.frame_id = image_out_ptr->header.frame_id;
  camera_pub_.publish(*(image_out_ptr->toImageMsg()), camera_info_out_);
}

void ImageUndistort::cameraCallback(
    const sensor_msgs::ImageConstPtr& image_msg_in,
    const sensor_msgs::CameraInfoConstPtr& camera_info_in) {
  camera_parameters_pair_ptr_->setCameraParameters(camera_info_in, true);
  if (!output_camera_info_from_yaml_) {
    camera_parameters_pair_ptr_->setCameraParameters(camera_info_in, false);
  }

  imageCallback(image_msg_in);
}
