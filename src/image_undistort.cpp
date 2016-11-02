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
  private_nh_.param("undistort_image", undistort_image_,
                    kDefaultUndistortImage);
  private_nh_.param("process_every_nth_frame", process_every_nth_frame_,
                    kDefaultProcessEveryNthFrame);

  // setup publishers
  if (process_image_) {
    std::string image_topic;
    if (output_camera_info_from_yaml_) {
      // load camera information from file (for correct operation when loading
      // from file this must be done before first call to updateCameraInfo)
      if (!loadCameraParameters(false, &camera_info_out_, &image_topic)) {
        ROS_FATAL("Loading of output camera parameters failed, exiting");
        ros::shutdown();
        exit(EXIT_FAILURE);
      }
    } else {
      image_topic = "output_image";
    }
    camera_pub_ = it_.advertiseCamera(image_topic, queue_size_);
  } else {
    camera_info_pub_ =
        nh_.advertise<sensor_msgs::CameraInfo>("cam_info", queue_size_);
  }

  // setup subscribers
  if (input_camera_info_from_yaml) {
    // load camera information from file
    sensor_msgs::CameraInfo loaded_camera_info;
    std::string image_topic;
    if (!loadCameraParameters(true, &loaded_camera_info, &image_topic)) {
      ROS_FATAL("Loading of input camera parameters failed, exiting");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }
    updateCameraInfo(loaded_camera_info);
    image_sub_ = it_.subscribe(image_topic, queue_size_,
                               &ImageUndistort::imageCallback, this);
  } else {
    camera_sub_ = it_.subscribeCamera("", queue_size_,
                                      &ImageUndistort::cameraCallback, this);
  }
}

void ImageUndistort::imageCallback(
    const sensor_msgs::ImageConstPtr& image_msg_in) {
  if (++frame_counter_ < process_every_nth_frame_) {
    return;
  }
  frame_counter_ = 0;

  if (!process_image_) {
    camera_info_in_.header.stamp = image_msg_in->header.stamp;
    camera_info_in_.header.frame_id = image_msg_in->header.frame_id;
    camera_info_pub_.publish(camera_info_in_);
  }
  cv_bridge::CvImageConstPtr image_in_ptr =
      cv_bridge::toCvShare(image_msg_in, "");
  cv_bridge::CvImagePtr image_out_ptr(
      new cv_bridge::CvImage(image_in_ptr->header, image_in_ptr->encoding));

  if (undistorter_ptr_) {
    undistorter_ptr_->undistortImage(image_in_ptr->image,
                                     &(image_out_ptr->image));
  } else {
    ROS_WARN("Attempted to undistort image before setting valid camera info");
    return;
  }

  camera_info_out_.header.stamp = image_out_ptr->header.stamp;
  camera_info_out_.header.frame_id = image_out_ptr->header.frame_id;
  camera_pub_.publish(*(image_out_ptr->toImageMsg()), camera_info_out_);
}

void ImageUndistort::cameraCallback(
    const sensor_msgs::ImageConstPtr& image_msg_in,
    const sensor_msgs::CameraInfoConstPtr& camera_info_in) {
  updateCameraInfo(*camera_info_in);
  imageCallback(image_msg_in);
}

void ImageUndistort::updateCameraInfo(
    const sensor_msgs::CameraInfo& camera_info) {
  // only update if camera parameters have changed
  if ((camera_info.K == camera_info_in_.K) &&
      (camera_info.width == camera_info_in_.width) &&
      (camera_info.height == camera_info_in_.height) &&
      (camera_info.distortion_model == camera_info_in_.distortion_model) &&
      (camera_info.D == camera_info_in_.D)) {
    return;
  }
  camera_info_in_ = camera_info;
  if (!output_camera_info_from_yaml_) {
    camera_info_out_ = camera_info_in_;
  }

  for (size_t i = 0; i < camera_info.D.size(); ++i) {
    if (!undistort_image_) {
      camera_info_in_.D[i] = 0;
      camera_info_out_.D[i] = camera_info.D[i];
    } else {
      camera_info_in_.D[i] = camera_info.D[i];
      camera_info_out_.D[i] = 0;
    }
  }

  Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> P_in(
      camera_info_in_.P.data());

  Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> P_out(
      camera_info_out_.P.data());

  cv::Size resolution(camera_info_out_.width, camera_info_out_.height);

  bool using_radtan;
  std::string lower_case_distortion_model = camera_info.distortion_model;
  transform(lower_case_distortion_model.begin(),
            lower_case_distortion_model.end(),
            lower_case_distortion_model.begin(), ::tolower);
  if ((lower_case_distortion_model == std::string("plumb bob")) ||
      (lower_case_distortion_model == std::string("radtan"))) {
    using_radtan = true;
  } else if (lower_case_distortion_model == std::string("equidistant")) {
    using_radtan = false;
  } else {
    ROS_ERROR_STREAM(
        "Unrecognized distortion model "
        << camera_info.distortion_model
        << ". Valid options are 'radtan', 'Plumb Bob' and 'equidistant'");
    undistorter_ptr_ = nullptr;
    return;
  }

  undistorter_ptr_ = std::make_shared<Undistorter>(
      resolution, P_in, P_out, using_radtan, camera_info_in_.D);
}

bool ImageUndistort::loadCameraParameters(
    const bool is_input, sensor_msgs::CameraInfo* loaded_camera_info,
    std::string* image_topic) {
  ROS_INFO("Loading camera parameters");

  std::string camera_name_space;
  if (is_input) {
    private_nh_.param("input_camera_name_space", camera_name_space,
                      kDefaultInputCameraNameSpace);
  } else {
    private_nh_.param("output_camera_name_space", camera_name_space,
                      kDefaultOutputCameraNameSpace);
  }

  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> K(
      loaded_camera_info->K.data());

  XmlRpc::XmlRpcValue K_in;
  bool K_set = false;
  if (private_nh_.getParam(camera_name_space + "/K", K_in)) {
    if (xmlRpcToMatrix(K_in, &K)) {
      K_set = true;
    } else {
      return false;
    }
  }

  std::vector<double> intrinsics_in;
  if (private_nh_.getParam(camera_name_space + "/intrinsics", intrinsics_in)) {
    if (K_set) {
      ROS_WARN(
          "Both K and intrinsics vector given, ignoring intrinsics vector");
    } else if (intrinsics_in.size() != 4) {
      ROS_FATAL("Intrinsics vector must have exactly 4 values (Fx,Fy,Cx,Cy)");
      return false;
    }

    K = Eigen::Matrix3d::Identity();
    K(0, 0) = intrinsics_in[0];
    K(1, 1) = intrinsics_in[1];
    K(0, 2) = intrinsics_in[2];
    K(1, 2) = intrinsics_in[3];
  } else if (!K_set) {
    ROS_FATAL("Could not find K or camera intrinsics vector");
    return false;
  }

  std::vector<double> resolution_in;
  if (private_nh_.getParam(camera_name_space + "/resolution", resolution_in)) {
    if (resolution_in.size() != 2) {
      ROS_FATAL("Resolution must have exactly 2 values (x,y)");
      return false;
    }
    loaded_camera_info->width = resolution_in[0];
    loaded_camera_info->height = resolution_in[1];
  } else {
    ROS_FATAL("Could not find camera resolution");
    return false;
  }

  if (is_input &&
      !private_nh_.getParam(camera_name_space + "/distortion_model",
                            loaded_camera_info->distortion_model)) {
    ROS_WARN("No distortion model given, assuming radtan");
  }

  if (private_nh_.getParam(camera_name_space + "/distortion_coeffs",
                           loaded_camera_info->D)) {
    if (!is_input) {
      ROS_WARN(
          "Distortion coefficients cannot be set for the output image, "
          "ignoring");
    }
  } else if (is_input) {
    ROS_WARN(
        "No distortion coefficients found, assuming images are "
        "undistorted");
    loaded_camera_info->D = std::vector<double>(0, 5);
  }

  // ensure D always has at least 5 elements
  while (loaded_camera_info->D.size() < 5) {
    loaded_camera_info->D.push_back(0);
  }

  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(
      loaded_camera_info->R.data());

  XmlRpc::XmlRpcValue R_in;
  if (private_nh_.getParam(camera_name_space + "/R", R_in)) {
    if (!xmlRpcToMatrix(R_in, &R)) {
      return false;
    }
  } else {
    R = Eigen::Matrix3d::Identity();
  }

  Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> P(
      loaded_camera_info->P.data());

  XmlRpc::XmlRpcValue P_in;
  if (private_nh_.getParam(camera_name_space + "/P", P_in)) {
    if (!xmlRpcToMatrix(P_in, &P)) {
      return false;
    }
  } else {
    P.topLeftCorner<3, 3>() = R * K;
    P.topRightCorner<3, 1>() = Eigen::Vector3d::Zero();
  }

  if (!P.bottomLeftCorner<1, 4>().isApprox(
          (Eigen::Matrix<double, 1, 4>() << 0, 0, 1, 0).finished())) {
    ROS_ERROR_STREAM("input P is not a projection matrix");
    return false;
  }

  // find rostopic name
  if (!private_nh_.getParam(camera_name_space + "/rostopic", *image_topic)) {
    if (is_input) {
      ROS_WARN("No rostopic found, setting topic to 'image'");
      *image_topic = "image";
    } else {
      *image_topic = "output_image";
    }
  }

  // Strip leading slash from rostopic name if the param is set.
  bool relative_camera_topic = false;
  private_nh_.param("relative_camera_topic", relative_camera_topic,
                    relative_camera_topic);
  if (relative_camera_topic && image_topic->front() == '/') {
    image_topic->erase(0, 1);
  }

  return true;
}
