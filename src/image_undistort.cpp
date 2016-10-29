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
  bool camera_info_from_yaml;
  private_nh_.param("camera_info_from_yaml", camera_info_from_yaml,
                    kDefaultCameraInfoFromYaml);
  private_nh_.param("queue_size", queue_size_, kQueueSize);
  if (queue_size_ < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size_ = 1;
  }
  private_nh_.param("process_image", process_image_, kDefaultProcessImage);
  private_nh_.param("undistort_image", undistort_image_,
                    kDefaultUndistortImage);
  private_nh_.param("modify_image_size", modify_image_size_,
                    kDefaultModifyImageSize);
  if (modify_image_size_) {
    if (private_nh_.getParam("output_image_size", output_image_size_)) {
      if (output_image_size_.size() != 2) {
        ROS_ERROR(
            "output_image_size requires exactly two values (width, height). "
            "Setting output image size to be the same as the input.");
        modify_image_size_ = false;
      }
    } else {
      ROS_ERROR(
          "modify_image_size is true, but output_image_size is not set. "
          "Setting output image size to be the same as the input.");
      modify_image_size_ = false;
    }
  }

  private_nh_.param("modify_intrinsics", modify_intrinsics_,
                    kDefaultModifyIntrinsics);
  if (modify_intrinsics_) {
    if (private_nh_.getParam("output_intrinsics", output_intrinsics_)) {
      if (output_image_size_.size() != 4) {
        ROS_ERROR(
            "output_intrinsics requires exactly four values (fx, fy, cx, cy). "
            "Setting output intrinsics to be the same as the input.");
        modify_intrinsics_ = false;
      }
    } else {
      ROS_ERROR(
          "modify_intrinsics is true, but output_intrinsics is not set. "
          "Setting output intrinsics to be the same as the input.");
      modify_intrinsics_ = false;
    }
  }

  private_nh_.param("process_every_nth_frame", process_every_nth_frame_,
                    kDefaultProcessEveryNthFrame);

  // setup subscribers
  if (camera_info_from_yaml) {
    // load camera information from file
    sensor_msgs::CameraInfo loaded_camera_info;
    if (!loadCameraParams(&loaded_camera_info)) {
      ROS_FATAL("Loading of camera parameters failed, exiting");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }
    updateCameraInfo(loaded_camera_info);
    image_sub_ = it_.subscribe("image", queue_size_,
                               &ImageUndistort::imageCallback, this);
  } else {
    camera_sub_ = it_.subscribeCamera("", queue_size_,
                                      &ImageUndistort::cameraCallback, this);
  }

  // setup publishers
  if (process_image_) {
    camera_pub_ = it_.advertiseCamera("output_image", queue_size_);
  } else {
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
    camera_info_in_.header.stamp = image_msg_in->header.stamp;
    camera_info_in_.header.frame_id = image_msg_in->header.frame_id;
    camera_info_pub_.publish(camera_info_in_);
  }

  cv_bridge::CvImageConstPtr image_in_ptr =
      cv_bridge::toCvShare(image_msg_in, "");
  cv_bridge::CvImagePtr image_out_ptr(
      new cv_bridge::CvImage(image_out_ptr->header, image_out_ptr->encoding));

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
    const sensor_msgs::CameraInfo& camera_info_in) {
  // only update if camera parameters have changed
  if ((camera_info_in.K == camera_info_in_.K) &&
      (camera_info_in.width == camera_info_in_.width) &&
      (camera_info_in.height == camera_info_in_.height) &&
      (camera_info_in.distortion_model == camera_info_in_.distortion_model) &&
      (camera_info_in.D == camera_info_in_.D)) {
    return;
  }

  camera_info_in_ = camera_info_in;
  camera_info_out_ = camera_info_in_;

  Eigen::Matrix<double, 3, 4> P_in =
      Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(
          camera_info_in.P.elems);

  cv::Size resolution;
  if (modify_image_size_) {
    resolution.width = output_image_size_[0];
    resolution.height = output_image_size_[1];
  } else {
    resolution.width = camera_info_in_.width;
    resolution.height = camera_info_in_.height;
  }
  camera_info_out_.width = resolution.width;
  camera_info_out_.height = resolution.height;

  bool using_radtan;
  if ((camera_info_in.distortion_model == std::string("Plumb Bob")) ||
      (camera_info_in.distortion_model == std::string("radtan"))) {
    using_radtan = true;
  } else if (camera_info_in.distortion_model == std::string("equidistant")) {
    using_radtan = false;
  } else {
    ROS_ERROR_STREAM(
        "Unrecognized distortion model "
        << camera_info_in.distortion_model
        << ". Valid options are 'radtan', 'Plumb Bob' and 'equidistant'");
    undistorter_ptr_ = nullptr;
    return;
  }

  std::vector<double> dist_vec = camera_info_in.D;
  if (!undistort_image_) {
    for (double& d : dist_vec) {
      d = 0;
    }
  } else {
    for (double& d : camera_info_out_.D) {
      d = 0;
    }
  }

  // set output rotation to identity
  camera_info_out_.R[0] = 1.0;
  camera_info_out_.R[1] = 0.0;
  camera_info_out_.R[2] = 0.0;
  camera_info_out_.R[3] = 1.0;
  camera_info_out_.R[4] = 0.0;
  camera_info_out_.R[5] = 0.0;
  camera_info_out_.R[6] = 1.0;
  camera_info_out_.R[7] = 0.0;
  camera_info_out_.R[8] = 0.0;

  // set intrinsics
  if (modify_intrinsics_) {
    camera_info_out_.K[0] = output_intrinsics_[0];
    camera_info_out_.K[1] = 0.0;
    camera_info_out_.K[2] = output_intrinsics_[2];
    camera_info_out_.K[3] = 0.0;
    camera_info_out_.K[4] = output_intrinsics_[1];
    camera_info_out_.K[5] = output_intrinsics_[3];
    camera_info_out_.K[6] = 0.0;
    camera_info_out_.K[7] = 0.0;
    camera_info_out_.K[8] = 1.0;
  }

  // set projection matrix to the same as K
  camera_info_out_.P[0] = camera_info_out_.K[0];
  camera_info_out_.P[1] = camera_info_out_.K[1];
  camera_info_out_.P[2] = camera_info_out_.K[2];
  camera_info_out_.P[3] = 0.0;
  camera_info_out_.P[4] = camera_info_out_.K[3];
  camera_info_out_.P[5] = camera_info_out_.K[4];
  camera_info_out_.P[6] = camera_info_out_.K[5];
  camera_info_out_.P[7] = 0.0;
  camera_info_out_.P[8] = camera_info_out_.K[6];
  camera_info_out_.P[9] = camera_info_out_.K[7];
  camera_info_out_.P[10] = camera_info_out_.K[8];
  camera_info_out_.P[11] = 0.0;

  Eigen::Matrix<double, 3, 4> P_out =
      Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(
          camera_info_out_.P.elems);

  undistorter_ptr_ = std::make_shared<Undistorter>(resolution, P_in, P_out,
                                                   using_radtan, dist_vec);
}

bool ImageUndistort::loadCameraParams(
    sensor_msgs::CameraInfo* loaded_camera_info) {
  ROS_INFO("Loading camera parameters");

  std::string camera_namespace;
  private_nh_.param("camera_namespace", camera_namespace,
                    kDefaultCameraNameSpace);

  std::vector<double> intrinsics_in;
  if (!nh_.getParam(camera_namespace + "/intrinsics", intrinsics_in)) {
    ROS_FATAL("Could not find camera intrinsics");
    return false;
  }
  if (intrinsics_in.size() != 4) {
    ROS_FATAL("Intrinsics must have exactly 4 values (Fx,Fy,Cx,Cy)");
    return false;
  }

  loaded_camera_info->K[0] = intrinsics_in[0];
  loaded_camera_info->K[1] = 0.0;
  loaded_camera_info->K[2] = intrinsics_in[2];
  loaded_camera_info->K[3] = 0.0;
  loaded_camera_info->K[4] = intrinsics_in[1];
  loaded_camera_info->K[5] = intrinsics_in[3];
  loaded_camera_info->K[6] = 0.0;
  loaded_camera_info->K[7] = 0.0;
  loaded_camera_info->K[8] = 1.0;

  std::vector<double> resolution_in;
  if (!nh_.getParam(camera_namespace + "/resolution", resolution_in)) {
    ROS_FATAL("Could not find camera resolution");
    return false;
  }
  if (resolution_in.size() != 2) {
    ROS_FATAL("Resolution must have exactly 2 values (x,y)");
    return false;
  }
  loaded_camera_info->width = resolution_in[0];
  loaded_camera_info->height = resolution_in[1];

  if (!nh_.getParam(camera_namespace + "/distortion_model",
                    loaded_camera_info->distortion_model)) {
    ROS_WARN("No distortion model given, assuming radtan");
  }

  if (!nh_.getParam(camera_namespace + "/distortion_coeffs",
                    loaded_camera_info->D)) {
    ROS_WARN(
        "No distortion coefficients found, assuming images are "
        "undistorted");
    loaded_camera_info->D = std::vector<double>(0, 5);
  }

  // ensure D always has at least 5 elements
  while (loaded_camera_info->D.size() < 5) {
    loaded_camera_info->D.push_back(0);
  }

  // assume no rotation
  loaded_camera_info->R[0] = 1.0;
  loaded_camera_info->R[1] = 0.0;
  loaded_camera_info->R[2] = 0.0;
  loaded_camera_info->R[3] = 1.0;
  loaded_camera_info->R[4] = 0.0;
  loaded_camera_info->R[5] = 0.0;
  loaded_camera_info->R[6] = 1.0;
  loaded_camera_info->R[7] = 0.0;
  loaded_camera_info->R[8] = 0.0;

  // set projection matrix to the same as K
  loaded_camera_info->P[0] = loaded_camera_info->K[0];
  loaded_camera_info->P[1] = loaded_camera_info->K[1];
  loaded_camera_info->P[2] = loaded_camera_info->K[2];
  loaded_camera_info->P[3] = 0.0;
  loaded_camera_info->P[4] = loaded_camera_info->K[3];
  loaded_camera_info->P[5] = loaded_camera_info->K[4];
  loaded_camera_info->P[6] = loaded_camera_info->K[5];
  loaded_camera_info->P[7] = 0.0;
  loaded_camera_info->P[8] = loaded_camera_info->K[6];
  loaded_camera_info->P[9] = loaded_camera_info->K[7];
  loaded_camera_info->P[10] = loaded_camera_info->K[8];
  loaded_camera_info->P[11] = 0.0;

  return true;
}
