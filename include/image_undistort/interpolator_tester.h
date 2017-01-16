#ifndef IMAGE_UNDISTORT_INTERPOLATOR_TESTER_H
#define IMAGE_UNDISTORT_INTERPOLATOR_TESTER_H

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <Eigen/Eigen>

#include "interpolator.h"

namespace image_undistort {

class Tester {
 public:
  Tester(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 private:
  static void generateRotationMap(
      const cv::Size& image_size, const double rotation_angle,
      Interpolator::DistortionMap* distortion_map_ptr);

  static void generateRotationMap(const cv::Size& image_size,
                                  const double rotation_angle, cv::Mat* map_x,
                                  cv::Mat* map_y);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
};
}

#endif