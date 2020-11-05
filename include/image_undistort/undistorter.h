#ifndef IMAGE_UNDISTORT_UNDISTORTER_H
#define IMAGE_UNDISTORT_UNDISTORTER_H

#include <Eigen/Dense>

#include <opencv2/imgproc/imgproc.hpp>

#include "image_undistort/camera_parameters.h"

namespace image_undistort {

class Undistorter {
 public:
  Undistorter(const CameraParametersPair& input_camera_parameters_pair);

  void undistortImage(const cv::Mat& image, cv::Mat* undistored_image);

  // get camera parameters used to build undistorter
  const CameraParametersPair& getCameraParametersPair();

  static void distortPixel(const Eigen::Matrix<double, 3, 3>& K_in,
                           const Eigen::Matrix<double, 3, 3>& R_in,
                           const Eigen::Matrix<double, 3, 4>& P_out,
                           const DistortionModel& distortion_model,
                           const std::vector<double>& D,
                           const Eigen::Vector2d& pixel_location,
                           Eigen::Vector2d* distorted_pixel_location);

 private:
  const CameraParametersPair used_camera_parameters_pair_;

  cv::Mat map_x_;
  cv::Mat map_y_;

  double empty_pixels_;
};
}

#endif
