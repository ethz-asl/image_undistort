#ifndef IMAGE_UNDISTORT_UNDISTORTER_H
#define IMAGE_UNDISTORT_UNDISTORTER_H

#include <Eigen/Eigen>

#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "image_undistort/camera_parameters.h"

namespace image_undistort {

class Undistorter {
 public:
  Undistorter(const CameraParametersPair& input_camera_parameters_pair);

  void undistortImage(const cv::Mat& image, cv::Mat* undistored_image);

  // get camera parameters used to build undistorter
  const CameraParametersPair& getCameraParametersPair();

  // generates a new output camera with fx = fy = (scale * (input_fx +
  // input_fy)/2, center point in the center of the image, R = I, and a
  // resolution that is as large as possible while having no empty pixels
  static void setOptimalOutputCameraParameters(
      const double scale, CameraParametersPair* camera_parameters_pair);

  static void distortPixel(const Eigen::Matrix<double, 3, 4>& P_in,
                           const Eigen::Matrix<double, 3, 4>& P_out,
                           const bool using_radtan,
                           const std::vector<double>& D,
                           const Eigen::Vector2d& pixel_location,
                           Eigen::Vector2d* distorted_pixel_location);

 private:
  const CameraParametersPair used_camera_parameters_pair_;

  #if (defined(CV_VERSION_EPOCH) && CV_VERSION_EPOCH == 2)
    cv::Mat map_x_;
    cv::Mat map_y_;
  #else
    cv::UMat map_x_;
    cv::UMat map_y_;
  #endif

  double empty_pixels_;
};
}

#endif
