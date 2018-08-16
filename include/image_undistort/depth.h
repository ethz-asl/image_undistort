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
constexpr int kDepthQueueSize = 10;
// small number used to check things are approximately equal
constexpr double kDelta = 0.000000001;
// stereo parameters
constexpr int kPreFilterCap = 31;
constexpr int kSADWindowSize = 11;
constexpr int kMinDisparity = 0;
constexpr int kNumDisparities = 64;
constexpr int kUniquenessRatio = 0;
constexpr int kSpeckleRange = 3;
constexpr int kSpeckleWindowSize = 500;
// bm parameters
constexpr int kTextureThreshold = 0;
const std::string kPreFilterType = "xsobel";
constexpr int kPreFilterSize = 9;
// sgbm parameters
constexpr bool kUseSGBM = false;
constexpr int kP1 = 120;
constexpr int kP2 = 240;
constexpr int kDisp12MaxDiff = -1;
constexpr bool kUseHHMode = false;

constexpr bool kDoMedianBlur = true;

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

  static void fillDisparityFromSide(const cv::Mat& input_disparity,
                                    const cv::Mat& valid, const bool& from_left,
                                    cv::Mat* filled_disparity);

  void bulidFilledDisparityImage(const cv::Mat& input_disparity,
                                 cv::Mat* disparity_filled,
                                 cv::Mat* input_valid) const;

  void calcDisparityImage(const sensor_msgs::ImageConstPtr& first_image_msg_in,
                          const sensor_msgs::ImageConstPtr& second_image_msg_in,
                          cv_bridge::CvImagePtr disparity_ptr) const;

  void calcPointCloud(const cv::Mat& input_disparity, const cv::Mat& left_image,
                      const double baseline, const double focal_length,
                      const int cx, const int cy,
                      pcl::PointCloud<pcl::PointXYZRGB>* pointcloud,
                      pcl::PointCloud<pcl::PointXYZRGB>* freespace_pointcloud);

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
  ros::Publisher freespace_pointcloud_pub_;

  // filters
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
      sensor_msgs::CameraInfo>
      CameraSyncPolicy;
  message_filters::Synchronizer<CameraSyncPolicy> camera_sync_;

  // stereo parameters
  int pre_filter_cap_;
  int sad_window_size_;
  int min_disparity_;
  int num_disparities_;
  int uniqueness_ratio_;
  int speckle_range_;
  int speckle_window_size_;

  // bm parameters
  int texture_threshold_;
  int pre_filter_type_;
  int pre_filter_size_;

  // sgbm parameters
  bool use_sgbm_;
  int p1_;
  int p2_;
  int disp_12_max_diff_;
  bool use_mode_HH_;

  bool do_median_blur_;
};
}

#endif
