#ifndef IMAGE_UNDISTORT_DISPARITY_KERNEL_H
#define IMAGE_UNDISTORT_DISPARITY_KERNEL_H

#define CL_HPP_MINIMUM_OPENCL_VERSION 110
#define CL_HPP_TARGET_OPENCL_VERSION 110

#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <CL/cl2.hpp>

namespace image_undistort {


class DisparityKernel {
 public:
  DisparityKernel();

  void computeDisparity(const cv::Mat& left_image, const cv::Mat& right_image, cv::Mat* disparity_image);

 private:

  cl::Program::Sources createSources(const KernelSourceInfo& source_info);
  cl::Device setupDevice();

  std::shared_ptr<KernelSourceInfo> source_info_ptr_;

  cl::Device device_;
  cl::Context context_;
  cl::Program program_;
  cl::CommandQueue queue_;

#endif
};
}
#endif