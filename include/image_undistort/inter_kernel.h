#ifndef IMAGE_UNDISTORT_INTER_KERNEL_H
#define IMAGE_UNDISTORT_INTER_KERNEL_H

#define CL_HPP_MINIMUM_OPENCL_VERSION 110
#define CL_HPP_TARGET_OPENCL_VERSION 110

#include <CL/cl2.hpp>

class InterKernel {
public:
  InterKernel();
private:
  std::vector<cl::Buffer> idx_maps_;
  std::vector<cl::Buffer> weight_maps_;

  cl::Device device_;
  cl::Context context_;
  cl::Program program_;
  cl::CommandQueue queue_;

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

  cv::Mat map_x_;
  cv::Mat map_y_;

  std::shared_ptr<cl::Buffer> map_e1_;
  std::shared_ptr<cl::Buffer> map_e2_;
  std::shared_ptr<cl::Buffer> map_f1_;
  std::shared_ptr<cl::Buffer> map_f2_;

  std::shared_ptr<cl::Program> cl_program_;
  std::shared_ptr<cl::CommandQueue> cl_queue_;
  std::shared_ptr<cl::Context> cl_context_;

  bool empty_pixels_;
};
}

#endif
