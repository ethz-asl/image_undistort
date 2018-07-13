#include "image_undistort/depth.h"

namespace image_undistort {

// needed so that topic filters can be initialized in the constructor
int Depth::getQueueSize() const {
  int queue_size;
  nh_private_.param("queue_size", queue_size, kQueueSize);
  if (queue_size < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size = 1;
  }
  return queue_size;
}

Depth::Depth(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      it_(nh_),
      queue_size_(getQueueSize()),
      first_image_sub_(it_, "rect/first/image", queue_size_),
      second_image_sub_(it_, "rect/second/image", queue_size_),
      first_camera_info_sub_(nh_, "rect/first/camera_info", queue_size_),
      second_camera_info_sub_(nh_, "rect/second/camera_info", queue_size_),
      camera_sync_(CameraSyncPolicy(queue_size_), first_image_sub_,
                   second_image_sub_, first_camera_info_sub_,
                   second_camera_info_sub_) {
  DepthGenerator::Config config;

  std::string pre_filter_type_string;
  nh_private_.param("pre_filter_type", pre_filter_type_string,
                    std::string("xsobel"));

  if (pre_filter_type_string == std::string("xsobel")) {
    config.pre_filter_type = cv::StereoBM::PREFILTER_XSOBEL;
  } else if (pre_filter_type_string == std::string("normalized_response")) {
    config.pre_filter_type = cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE;
  } else {
    throw std::runtime_error(
        "Unrecognized prefilter type, choices are 'xsobel' or "
        "'normalized_response'");
  }

  // general stereo parameters
  nh_private_.param("min_disparity", config.min_disparity,
                    config.min_disparity);
  nh_private_.param("num_disparities", config.num_disparities,
                    config.num_disparities);
  nh_private_.param("pre_filter_cap", config.pre_filter_cap,
                    config.pre_filter_cap);
  nh_private_.param("uniqueness_ratio", config.uniqueness_ratio,
                    config.uniqueness_ratio);
  nh_private_.param("speckle_range", config.speckle_range,
                    config.speckle_range);
  nh_private_.param("speckle_window_size", config.speckle_window_size,
                    config.speckle_window_size);
  nh_private_.param("sad_window_size", config.sad_window_size,
                    config.sad_window_size);

  // bm parameters
  nh_private_.param("texture_threshold", config.texture_threshold,
                    config.texture_threshold);
  nh_private_.param("pre_filter_size", config.pre_filter_size,
                    config.pre_filter_size);

  // sgbm parameters
  nh_private_.param("use_sgbm", config.use_sgbm, config.use_sgbm);
  nh_private_.param("p1", config.p1, config.p1);
  nh_private_.param("p2", config.p2, config.p2);
  nh_private_.param("disp_12_max_diff", config.disp_12_max_diff,
                    config.disp_12_max_diff);
  nh_private_.param("use_mode_HH", config.use_mode_HH, config.use_mode_HH);

  nh_private_.param("do_median_blur", config.do_median_blur,
                    config.do_median_blur);

  camera_sync_.registerCallback(
      boost::bind(&Depth::camerasCallback, this, _1, _2, _3, _4));

  disparity_pub_ = it_.advertise("disparity/image", queue_size_);
  pointcloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("pointcloud", queue_size_);
  freespace_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "freespace_pointcloud", queue_size_);
}

void Depth::camerasCallback(
    const sensor_msgs::ImageConstPtr& first_image_msg,
    const sensor_msgs::ImageConstPtr& second_image_msg,
    const sensor_msgs::CameraInfoConstPtr& first_camera_info,
    const sensor_msgs::CameraInfoConstPtr& second_camera_info) {
  double baseline, focal_length;
  bool first_is_left;
  int cx, cy;

  if (!depth_gen_->processCameraInfo(*first_camera_info, *second_camera_info,
                                     &baseline, &focal_length, &first_is_left,
                                     &cx, &cy)) {
    ROS_ERROR("Camera info processing failed, skipping disparity generation");
    return;
  }

  sensor_msgs::ImageConstPtr left_image_msg;
  sensor_msgs::ImageConstPtr right_image_msg;

  if (first_is_left) {
    left_image_msg = first_image_msg;
    right_image_msg = second_image_msg;
  } else {
    left_image_msg = second_image_msg;
    right_image_msg = first_image_msg;
  }

  cv::Mat disparity_image =
      cv::Mat(left_image_msg->height, left_image_msg->width, CV_16S);

  cv_bridge::CvImagePtr disparity_ptr(new cv_bridge::CvImage(
      left_image_msg->header, "mono16", disparity_image));
  cv_bridge::CvImageConstPtr left_ptr =
      cv_bridge::toCvShare(left_image_msg, "mono8");
  cv_bridge::CvImageConstPtr right_ptr =
      cv_bridge::toCvShare(right_image_msg, "mono8");

  depth_gen_->calcDisparityImage(left_ptr->image, right_ptr->image,
                                 &(disparity_ptr->image));
  disparity_pub_.publish(*(disparity_ptr->toImageMsg()));

  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB> freespace_pointcloud;

  left_ptr = cv_bridge::toCvShare(left_image_msg);
  depth_gen_->calcPointCloud(disparity_ptr->image, left_ptr->image, baseline,
                             focal_length, cx, cy, &pointcloud,
                             &freespace_pointcloud);

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  pointcloud_msg.header = left_image_msg->header;
  pointcloud_pub_.publish(pointcloud_msg);

  sensor_msgs::PointCloud2 freespace_pointcloud_msg;
  pcl::toROSMsg(freespace_pointcloud, freespace_pointcloud_msg);
  freespace_pointcloud_msg.header = left_image_msg->header;
  freespace_pointcloud_pub_.publish(freespace_pointcloud_msg);
}
}  // namespace image_undistort
