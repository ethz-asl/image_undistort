#ifndef IMAGE_UNDISTORT_DISPARITY_H
#define IMAGE_UNDISTORT_DISPARITY_H

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace image_undistort {

// Default values

// queue size
constexpr int kQueueSize = 10;
// small number used to check things are approximately equal
constexpr double kDelta = 0.000000001;
// stereobm parameters
const std::string kPreFilterType = "xsobel";
constexpr int kPreFilterSize = 9;
constexpr int kPreFilterCap = 31;
constexpr int kSADWindowSize = 21;
constexpr int kMinDisparity = 0;
constexpr int kNumDisparities = 64;
constexpr int kTextureThreshold = 10;
constexpr int kUniquenessRatio = 15;
constexpr int kSpeckleRange = 0;
constexpr int kSpeckleWindowSize = 0;

constexpr bool kEnableWLSFilter = false;

class Depth {
 public:
  Depth(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void camerasCallback(
      const sensor_msgs::ImageConstPtr& first_image_msg_in,
      const sensor_msgs::ImageConstPtr& second_image_msg_in,
      const sensor_msgs::CameraInfoConstPtr& first_camera_info_msg_in,
      const sensor_msgs::CameraInfoConstPtr& second_camera_info_msg_in);

 private:
  int getQueueSize() const;

  static bool ApproxEq(double A, double B);

  bool processCameraInfo(
      const sensor_msgs::CameraInfoConstPtr& first_camera_info,
      const sensor_msgs::CameraInfoConstPtr& second_camera_info,
      double* baseline, double* focal_length, bool* first_is_left, int* cx,
      int* cy);

  void calcDisparityImage(const sensor_msgs::ImageConstPtr& first_image_msg_in,
                          const sensor_msgs::ImageConstPtr& second_image_msg_in,
                          cv_bridge::CvImagePtr disparity_ptr) const;

  void calcPointCloud(const cv_bridge::CvImagePtr disparity_ptr,
                      const sensor_msgs::ImageConstPtr& left_image_msg,
                      const double baseline, const double focal_length,
                      const int cx, const int cy,
                      pcl::PointCloud<pcl::PointXYZRGB>* pointcloud);

  // nodes
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  image_transport::ImageTransport it_;

  int queue_size_;

  // subscribers
  image_transport::SubscriberFilter first_image_sub_;
  image_transport::SubscriberFilter second_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> first_camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> second_camera_info_sub_;

  // publishers
  image_transport::Publisher disparity_pub_;
  ros::Publisher pointcloud_pub_;

  // filters
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
      sensor_msgs::CameraInfo>
      CameraSyncPolicy;
  message_filters::Synchronizer<CameraSyncPolicy> camera_sync_;

  // stereobm parameters
  int pre_filter_type_;
  int pre_filter_size_;
  int pre_filter_cap_;
  int sad_window_size_;
  int min_disparity_;
  int num_disparities_;
  int texture_threshold_;
  int uniqueness_ratio_;
  int speckle_range_;
  int speckle_window_size_;
};
}

#endif
