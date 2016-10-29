#include <image_undistort/image_undistort.h>
#include <image_undistort/undistorter.h>

ImageUndistort::ImageUndistort(const ros::NodeHandle& nh,
                               const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), it_(nh_), undistorter_ptr_(nullptr) {
  bool cam_info_from_yaml;
  private_nh_.param("cam_info_from_yaml", cam_info_from_yaml,
                    kDefaultCamInfoFromYaml);

  std::string cam_ns;
  private_nh_.param("cam_ns", cam_ns, kDefaultCamNameSpace);

  private_nh_.param("zoom", zoom_, kDefaultZoom);

  if (cam_info_from_yaml) {
    sensor_msgs::CameraInfo new_raw_cam_info;
    if (!loadCamParams(cam_ns, &new_raw_cam_info)) {
      ROS_FATAL("Loading of camera parameters failed, exiting");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }
    updateCamInfo(new_raw_cam_info);
    raw_image_sub_ = it_.subscribe("image", kQueueSize,
                                   &ImageUndistort::imageCallback, this);
  } else {
    raw_camera_sub_ = it_.subscribeCamera(
        cam_ns, kQueueSize, &ImageUndistort::cameraCallback, this);
  }

  undistorted_camera_pub_ = it_.advertiseCamera("undistorted", kQueueSize);
}

void ImageUndistort::imageCallback(
    const sensor_msgs::ImageConstPtr& raw_image_msg) {
  cv_bridge::CvImageConstPtr raw_image_ptr =
      cv_bridge::toCvShare(raw_image_msg, "");
  cv_bridge::CvImagePtr undistorted_image_ptr(
      new cv_bridge::CvImage(raw_image_ptr->header, raw_image_ptr->encoding));

  if (undistorter_ptr_) {
    undistorter_ptr_->undistortImage(raw_image_ptr->image,
                                     &(undistorted_image_ptr->image));
  } else {
    ROS_WARN("Attempted to undistort image before setting valid camera info");
    return;
  }

  undistorted_camera_pub_.publish(*(undistorted_image_ptr->toImageMsg()),
                                  undistorted_cam_info_);
}

void ImageUndistort::cameraCallback(
    const sensor_msgs::ImageConstPtr& raw_image_msg,
    const sensor_msgs::CameraInfoConstPtr& new_raw_cam_info) {
  updateCamInfo(*new_raw_cam_info);
  imageCallback(raw_image_msg);
}

void ImageUndistort::updateCamInfo(
    const sensor_msgs::CameraInfo& new_raw_cam_info) {
  // only update if camera parameters have changed
  if ((new_raw_cam_info.K == raw_cam_info_.K) &&
      (new_raw_cam_info.width == raw_cam_info_.width) &&
      (new_raw_cam_info.height == raw_cam_info_.height) &&
      (new_raw_cam_info.distortion_model == raw_cam_info_.distortion_model) &&
      (new_raw_cam_info.D == raw_cam_info_.D)) {
    return;
  }

  Eigen::Matrix3d K =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          new_raw_cam_info.K.elems);

  const cv::Size resolution(new_raw_cam_info.width, new_raw_cam_info.height);

  bool using_radtan;
  if ((new_raw_cam_info.distortion_model == std::string("Plumb Bob")) ||
      (new_raw_cam_info.distortion_model == std::string("radtan"))) {
    using_radtan = true;
  } else if (new_raw_cam_info.distortion_model == std::string("equidistant")) {
    using_radtan = false;
  } else {
    ROS_ERROR_STREAM(
        "Unrecognized distortion model "
        << new_raw_cam_info.distortion_model
        << ". Valid options are 'radtan', 'Plumb Bob' and 'equidistant'");
    undistorter_ptr_ = nullptr;
    return;
  }

  undistorter_ptr_ = std::make_shared<Undistorter>(
      K, new_raw_cam_info.D, resolution, using_radtan, zoom_);

  undistorted_cam_info_ = new_raw_cam_info;
  undistorted_cam_info_.K[0] *= zoom_;
  undistorted_cam_info_.K[4] *= zoom_;
  for (double& d : undistorted_cam_info_.D) {
    d = 0;
  }
}

bool ImageUndistort::loadCamParams(const std::string& cam_ns,
                                   sensor_msgs::CameraInfo* new_raw_cam_info) {
  ROS_INFO("Loading camera parameters");

  std::vector<double> intrinsics_in;
  if (!nh_.getParam(cam_ns + "/intrinsics", intrinsics_in)) {
    ROS_FATAL("Could not find camera intrinsics");
    return false;
  }
  if (intrinsics_in.size() != 4) {
    ROS_FATAL("Intrinsics must have exactly 4 values (Fx,Fy,Cx,Cy)");
    return false;
  }

  new_raw_cam_info->K[0] = intrinsics_in[0];
  new_raw_cam_info->K[1] = 0.0;
  new_raw_cam_info->K[2] = intrinsics_in[2];
  new_raw_cam_info->K[3] = 0.0;
  new_raw_cam_info->K[4] = intrinsics_in[1];
  new_raw_cam_info->K[5] = intrinsics_in[3];
  new_raw_cam_info->K[6] = 0.0;
  new_raw_cam_info->K[7] = 0.0;
  new_raw_cam_info->K[8] = 1.0;

  std::vector<double> resolution_in;
  if (!nh_.getParam(cam_ns + "/resolution", resolution_in)) {
    ROS_FATAL("Could not find camera resolution");
    return false;
  }
  if (resolution_in.size() != 2) {
    ROS_FATAL("Resolution must have exactly 2 values (x,y)");
    return false;
  }
  new_raw_cam_info->width = resolution_in[0];
  new_raw_cam_info->height = resolution_in[1];

  if (!nh_.getParam(cam_ns + "/distortion_model",
                    new_raw_cam_info->distortion_model)) {
    ROS_WARN("No distortion model given, assuming radtan");
  }

  if (!nh_.getParam(cam_ns + "/distortion_coeffs", new_raw_cam_info->D)) {
    ROS_WARN(
        "No distortion coefficients found, assuming images are undistorted");
    new_raw_cam_info->D = std::vector<double>(0, 5);
  }

  // ensure D always has at least 5 elements
  while (new_raw_cam_info->D.size() < 5) {
    new_raw_cam_info->D.push_back(0);
  }

  // assume no rotation
  new_raw_cam_info->R[0] = 1.0;
  new_raw_cam_info->R[1] = 0.0;
  new_raw_cam_info->R[2] = 0.0;
  new_raw_cam_info->R[3] = 1.0;
  new_raw_cam_info->R[4] = 0.0;
  new_raw_cam_info->R[5] = 0.0;
  new_raw_cam_info->R[6] = 1.0;
  new_raw_cam_info->R[7] = 0.0;
  new_raw_cam_info->R[8] = 0.0;

  // set projection matrix to the same as K
  new_raw_cam_info->P[0] = new_raw_cam_info->K[0];
  new_raw_cam_info->P[1] = new_raw_cam_info->K[1];
  new_raw_cam_info->P[2] = new_raw_cam_info->K[2];
  new_raw_cam_info->P[3] = 0.0;
  new_raw_cam_info->P[4] = new_raw_cam_info->K[3];
  new_raw_cam_info->P[5] = new_raw_cam_info->K[4];
  new_raw_cam_info->P[6] = new_raw_cam_info->K[5];
  new_raw_cam_info->P[7] = 0.0;
  new_raw_cam_info->P[8] = new_raw_cam_info->K[6];
  new_raw_cam_info->P[9] = new_raw_cam_info->K[7];
  new_raw_cam_info->P[10] = new_raw_cam_info->K[8];
  new_raw_cam_info->P[11] = 0.0;

  return true;
}
