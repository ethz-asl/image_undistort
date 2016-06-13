#include <image_undistort/image_undistort.h>
#include <image_undistort/undistorter.h>

ImageUndistort::ImageUndistort(const ros::NodeHandle& nh,
                               const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), undistorter_ptr_(nullptr) {
  image_sub_ = nh_.subscribe("image", kQueueSize,
                             &ImageUndistort::newFrameCallback, this);
  cam_info_sub_ = nh_.subscribe("cam_info", kQueueSize,
                                &ImageUndistort::camInfoCallback, this);
  image_pub_ = nh_.advertise<sensor_msgs::Image>("undistorted_image", 0);
}

void ImageUndistort::newFrameCallback(
    const sensor_msgs::ImageConstPtr& image_msg) {
  cv::Mat image_distorted, image_undistorted;

  image_distorted = cv_bridge::toCvCopy(image_msg, "")->image;

  if (undistorter_ptr_) {
    undistorter_ptr_->undistortImage(image_distorted, &image_undistorted);
  } else {
    ROS_WARN("Attempted to undistort image before setting valid camera info");
    return;
  }

  cv_bridge::CvImage out_msg;
  out_msg.header = image_msg->header;
  out_msg.encoding = image_msg->encoding;
  out_msg.image = image_undistorted;

  image_pub_.publish(out_msg);
}

void ImageUndistort::camInfoCallback(
    const sensor_msgs::CameraInfoConstPtr& camera_msg) {
  Eigen::Matrix3d K;
  for (size_t i = 0; i < camera_msg->K.size(); ++i) {
    K(i) = camera_msg->K[i];
  }

  const cv::Size resolution(camera_msg->width, camera_msg->height);

  bool using_radtan;
  if (camera_msg->distortion_model == std::string("radtan")) {
    using_radtan = true;
  } else if (camera_msg->distortion_model == std::string("equidistant")) {
    using_radtan = false;
  } else {
    ROS_ERROR_STREAM("Unrecognized distortion model "
                     << camera_msg->distortion_model
                     << ". Valid options are radtan and equidistant");
    undistorter_ptr_ = nullptr;
    return;
  }

  double zoom;
  private_nh_.param("zoom", zoom, kDefaultZoom);

  undistorter_ptr_ = std::make_shared<Undistorter>(K, camera_msg->D, resolution,
                                                   using_radtan, zoom);
}