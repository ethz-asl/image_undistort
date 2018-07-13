#ifndef IMAGE_UNDISTORT_DEPTH_GENERATOR_H
#define IMAGE_UNDISTORT_DEPTH_GENERATOR_H

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace image_undistort {

// small number used to check things are approximately equal
constexpr double kDelta = 1e-8;

class DepthGenerator {
 public:
  struct Config {
    // stereo parameters
    int pre_filter_cap = 31;
    int sad_window_size = 11;
    int min_disparity = 0;
    int num_disparities = 64;
    int uniqueness_ratio = 0;
    int speckle_range = 3;
    int speckle_window_size = 500;

    // bm parameters
    int texture_threshold = 0;
    int pre_filter_type = cv::StereoBM::PREFILTER_XSOBEL;
    int pre_filter_size = 9;

    // sgbm parameters
    bool use_sgbm = false;
    int p1 = 120;
    int p2 = 240;
    int disp_12_max_diff = -1;
    bool use_mode_HH = false;

    bool do_median_blur = true;
  };

  DepthGenerator(const Config& config) : config_(config){}

  void calcDisparityImage(const cv::Mat& first_image,
                          const cv::Mat& second_image,
                          cv::Mat* disparity_ptr) const;

  void calcPointCloud(const cv::Mat& input_disparity, const cv::Mat& left_image,
                      const double baseline, const double focal_length,
                      const int cx, const int cy,
                      pcl::PointCloud<pcl::PointXYZRGB>* pointcloud,
                      pcl::PointCloud<pcl::PointXYZRGB>* freespace_pointcloud);

  static bool processCameraInfo(
      const sensor_msgs::CameraInfo& first_camera_info,
      const sensor_msgs::CameraInfo& second_camera_info, double* baseline,
      double* focal_length, bool* first_is_left, int* cx, int* cy);

 private:
  int getQueueSize() const;

  static bool ApproxEq(double A, double B);

  static void fillDisparityFromSide(const cv::Mat& input_disparity,
                                    const cv::Mat& valid, const bool& from_left,
                                    cv::Mat* filled_disparity);

  void bulidFilledDisparityImage(const cv::Mat& input_disparity,
                                 cv::Mat* disparity_filled,
                                 cv::Mat* input_valid) const;

  // holds all stereo matching parameters
  Config config_;
};
}  // namespace image_undistort

#endif
