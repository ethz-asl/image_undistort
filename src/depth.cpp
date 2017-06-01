#include "image_undistort/depth.h"

#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"

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
  std::string pre_filter_type_string;
  nh_private_.param("pre_filter_type", pre_filter_type_string, kPreFilterType);
  if (pre_filter_type_string == "xsobel") {
    pre_filter_type_ = cv::StereoBM::PREFILTER_XSOBEL;
  } else if (pre_filter_type_string == "normalized_response") {
    pre_filter_type_ = cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE;
  } else {
    throw std::runtime_error(
        "Unrecognized prefilter type, choices are 'xsobel' or "
        "'normalized_response'");
  }
  nh_private_.param("pre_filter_size", pre_filter_size_, kPreFilterSize);
  nh_private_.param("pre_filter_cap", pre_filter_cap_, kPreFilterCap);
  nh_private_.param("sad_window_size", sad_window_size_, kSADWindowSize);
  nh_private_.param("min_disparity", min_disparity_, kMinDisparity);
  nh_private_.param("num_disparities", num_disparities_, kNumDisparities);
  nh_private_.param("texture_threshold", texture_threshold_, kTextureThreshold);
  nh_private_.param("uniqueness_ratio", uniqueness_ratio_, kUniquenessRatio);
  nh_private_.param("speckle_range", speckle_range_, kSpeckleRange);
  nh_private_.param("speckle_window_size", speckle_window_size_,
                    kSpeckleWindowSize);

  nh_private_.param("enable_wls_filter", enable_wls_filter_,
                    kEnableWLSFilter);

  camera_sync_.registerCallback(
      boost::bind(&Depth::camerasCallback, this, _1, _2, _3, _4));

  disparity_pub_ = it_.advertise("disparity/image", queue_size_);
  pointcloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("pointcloud", queue_size_);
}

void Depth::calcPointCloud(const cv_bridge::CvImagePtr disparity_ptr,
                           const sensor_msgs::ImageConstPtr& left_image_msg,
                           const double baseline, const double focal_length,
                           const int cx, const int cy,
                           pcl::PointCloud<pcl::PointXYZRGB>* pointcloud) {
  pointcloud->clear();

  cv_bridge::CvImageConstPtr left_ptr = cv_bridge::toCvShare(left_image_msg);

  if (left_ptr->image.depth() != CV_8U) {
    ROS_ERROR(
        "Pointcloud generation is currently only supported on 8 bit images");
    return;
  }

  for (int y_pixels = 0; y_pixels < disparity_ptr->image.rows; ++y_pixels) {
    for (int x_pixels = 0; x_pixels < disparity_ptr->image.cols;
         ++x_pixels) {
      int disparity_idx = x_pixels + disparity_ptr->image.cols * y_pixels;

      const uint16_t& disparity_value =
          reinterpret_cast<uint16_t*>(disparity_ptr->image.data)[disparity_idx];

      if (disparity_value <= 0) {
        continue;
      }

      pcl::PointXYZRGB point;

      // the 16* is needed as opencv stores disparity maps as 16 * the true
      // values
      point.z =
          (16 * focal_length * baseline) / static_cast<double>(disparity_value);
      point.x = point.z * (x_pixels - cx) / focal_length;
      point.y = point.z * (y_pixels - cy) / focal_length;

      point.b = reinterpret_cast<uint8_t*>(left_ptr->image.data)[disparity_idx];

      if (left_ptr->image.channels() >= 3) {
        point.g = reinterpret_cast<uint8_t*>(
            left_ptr->image
                .data)[disparity_idx +
                       disparity_ptr->image.rows * disparity_ptr->image.cols];
        point.r = reinterpret_cast<uint8_t*>(
            left_ptr->image.data)[disparity_idx +
                                  2 * disparity_ptr->image.rows *
                                      disparity_ptr->image.cols];
      } else {
        point.g = point.b;
        point.r = point.b;
      }

      pointcloud->push_back(point);
    }
  }
}

void Depth::calcDisparityImage(
    const sensor_msgs::ImageConstPtr& left_image_msg,
    const sensor_msgs::ImageConstPtr& right_image_msg,
    cv_bridge::CvImagePtr disparity_ptr) const {
  cv_bridge::CvImageConstPtr left_ptr =
      cv_bridge::toCvShare(left_image_msg, "mono8");

  cv_bridge::CvImageConstPtr right_ptr =
      cv_bridge::toCvShare(right_image_msg, "mono8");

  cv::Mat left_disp =
      cv::Mat(left_ptr->image.rows, left_ptr->image.cols, CV_16S);
  cv::Mat right_disp =
      cv::Mat(left_ptr->image.rows, left_ptr->image.cols, CV_16S);

  cv::Ptr<cv::StereoBM> left_matcher =
      cv::StereoBM::create(num_disparities_, sad_window_size_);

  left_matcher->setPreFilterType(pre_filter_type_);
  left_matcher->setPreFilterCap(pre_filter_cap_);
  left_matcher->setMinDisparity(min_disparity_);
  left_matcher->setTextureThreshold(texture_threshold_);
  left_matcher->setUniquenessRatio(uniqueness_ratio_);
  left_matcher->setSpeckleRange(speckle_range_);
  left_matcher->setSpeckleWindowSize(speckle_window_size_);

  if (enable_wls_filter_) {
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter =
        cv::ximgproc::createDisparityWLSFilter(left_matcher);
    cv::Ptr<cv::StereoMatcher> right_matcher =
        cv::ximgproc::createRightMatcher(left_matcher);
    right_matcher->compute(right_ptr->image, left_ptr->image, right_disp);
    left_matcher->compute(left_ptr->image, right_ptr->image, left_disp);

    wls_filter->filter(left_disp, left_ptr->image, disparity_ptr->image,
                       right_disp);
  } else {
    left_matcher->compute(left_ptr->image, right_ptr->image,
                          disparity_ptr->image);
  }

  cv::medianBlur(disparity_ptr->image, disparity_ptr->image, 5);
}

bool Depth::processCameraInfo(
    const sensor_msgs::CameraInfoConstPtr& first_camera_info,
    const sensor_msgs::CameraInfoConstPtr& second_camera_info, double* baseline,
    double* focal_length, bool* first_is_left, int* cx, int* cy) {
  if (first_camera_info->height != second_camera_info->height) {
    ROS_ERROR("Image heights do not match");
    return false;
  }
  if (first_camera_info->width != second_camera_info->width) {
    ROS_ERROR("Image widths do not match");
    return false;
  }

  for (double d : first_camera_info->D) {
    if (!ApproxEq(d, 0)) {
      ROS_ERROR("First image has non-zero distortion");
      return false;
    }
  }
  for (double d : second_camera_info->D) {
    if (!ApproxEq(d, 0)) {
      ROS_ERROR("Second image has non-zero distortion");
      return false;
    }
  }

  for (size_t i = 0; i < 12; ++i) {
    if ((i != 3) &&
        !ApproxEq(first_camera_info->P[i], second_camera_info->P[i])) {
      ROS_ERROR("Image P matrices must match (excluding x offset)");
      return false;
    }
  }

  if (!ApproxEq(first_camera_info->P[1], 0) ||
      !ApproxEq(first_camera_info->P[4], 0)) {
    ROS_ERROR("Image P matrix contains skew");
    return false;
  }

  if (!ApproxEq(first_camera_info->P[0], first_camera_info->P[5])) {
    ROS_ERROR("Image P matrix has different values for Fx and Fy");
    return false;
  }

  if (first_camera_info->P[0] <= 0) {
    ROS_ERROR("Focal length must be greater than 0");
    return false;
  }

  if (!ApproxEq(first_camera_info->P[8], 0) ||
      !ApproxEq(first_camera_info->P[9], 0) ||
      !ApproxEq(first_camera_info->P[10], 1) ||
      !ApproxEq(first_camera_info->P[11], 0)) {
    ROS_ERROR("Image P matrix does not end in [0,0,1,0]");
    return false;
  }

  if (!ApproxEq(first_camera_info->P[7], 0)) {
    ROS_ERROR("P contains Y offset");
    return false;
  }

  *focal_length = first_camera_info->P[0];
  *baseline = (second_camera_info->P[3] - first_camera_info->P[3]) /
              first_camera_info->P[0];
  if (*baseline > 0) {
    *first_is_left = false;
  } else {
    *first_is_left = true;
    *baseline *= -1;
  }
  *cx = first_camera_info->P[2];
  *cy = first_camera_info->P[6];

  return true;
}

bool Depth::ApproxEq(double A, double B) { return (std::abs(A - B) <= kDelta); }

void Depth::camerasCallback(
    const sensor_msgs::ImageConstPtr& first_image_msg,
    const sensor_msgs::ImageConstPtr& second_image_msg,
    const sensor_msgs::CameraInfoConstPtr& first_camera_info,
    const sensor_msgs::CameraInfoConstPtr& second_camera_info) {
  double baseline, focal_length;
  bool first_is_left;
  int cx, cy;

  if (!processCameraInfo(first_camera_info, second_camera_info, &baseline,
                         &focal_length, &first_is_left, &cx, &cy)) {
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

  calcDisparityImage(left_image_msg, right_image_msg, disparity_ptr);
  disparity_pub_.publish(*(disparity_ptr->toImageMsg()));

  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  calcPointCloud(disparity_ptr, left_image_msg, baseline, focal_length, cx, cy,
                 &pointcloud);

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  pointcloud_msg.header = left_image_msg->header;
  pointcloud_pub_.publish(pointcloud_msg);
}
}
