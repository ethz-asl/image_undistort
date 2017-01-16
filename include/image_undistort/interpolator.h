#ifndef IMAGE_UNDISTORT_INTERPOLATOR_H
#define IMAGE_UNDISTORT_INTERPOLATOR_H

#define CL_HPP_MINIMUM_OPENCL_VERSION 110
#define CL_HPP_TARGET_OPENCL_VERSION 110

#include <cv.h>
#include <ros/ros.h>
#include <CL/cl2.hpp>
#include <Eigen/Eigen>

namespace image_undistort {

class Interpolator {
 public:
  typedef Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, Eigen::Dynamic>
      DistortionMap;

  Interpolator(const cv::Size& input_image_size,
               const DistortionMap& distortion_map);

  void Interpolate(const cv::Mat& image, cv::Mat* undistorted_image);

 private:
  cl::Program::Sources createSources();
  cl::Device setupDevice();

  std::vector<cl_uint> idx_map_;
  cl::Buffer idx_buffer_;

  std::vector<cl_short> offset_map_;
  cl::Buffer offset_buffer_;

  std::vector<cl_uchar> weight_map_;
  cl::Buffer weight_buffer_;

  cl::Device device_;
  cl::Context context_;
  cl::Program program_;
  cl::CommandQueue queue_;


  cv::Size output_image_size_;
};
}

#endif