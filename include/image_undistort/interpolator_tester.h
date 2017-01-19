#ifndef IMAGE_UNDISTORT_INTERPOLATOR_TESTER_H
#define IMAGE_UNDISTORT_INTERPOLATOR_TESTER_H

#include <cv.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "interpolator.h"

namespace image_undistort {

class Tester {
 public:
  Tester(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 private:

  static void interpolateNearestLinear(const cv::Mat input,
                                      const Interpolator::DistortionMap map,
                                      cv::Mat* output);

  static float CubicHermite(float A, float B, float C, float D, float t);

  static void interpolateBicubic(const cv::Mat input,
                                 const Interpolator::DistortionMap map,
                                 cv::Mat* output);

  static void interpolateBilinear(const cv::Mat input,
                                  const Interpolator::DistortionMap map,
                                  cv::Mat* output);

  static void interpolateNearestNeighbor(const cv::Mat input,
                                         const Interpolator::DistortionMap map,
                                         cv::Mat* output);

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