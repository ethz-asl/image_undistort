#ifndef IMAGE_UNDISTORT_INTERPOLATOR_H
#define IMAGE_UNDISTORT_INTERPOLATOR_H

#define CL_HPP_MINIMUM_OPENCL_VERSION 110
#define CL_HPP_TARGET_OPENCL_VERSION 110

#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>

#ifdef USE_OPENCL
#include <CL/cl2.hpp>
#endif

#include <Eigen/Eigen>

namespace image_undistort {

class KernelSourceInfo {
 public:
  KernelSourceInfo(const cv::Size& input_image_size, const int image_type,
                   const size_t number_channels, const bool empty_pixels);

  bool operator==(const KernelSourceInfo& B) const;

  bool operator!=(const KernelSourceInfo& B) const;

  const cv::Size input_image_size_;
  const int image_type_;
  const size_t number_channels_;
  const bool empty_pixels_;
};

class Interpolator {
 public:
  Interpolator(const cv::Mat& distortion_map_x,
               const cv::Mat& distortion_map_y);

  void Interpolate(const cv::Mat& image, const KernelSourceInfo& source_info,
                   cv::Mat* undistorted_image);

 private:
  cv::Mat distortion_map_x_;
  cv::Mat distortion_map_y_;

#ifdef USE_OPENCL

  // super ugly byte manipulation function
  template <typename T>
  static void addWeights(const Eigen::Vector2d remainder,
                         std::vector<cl_uchar>* weight_map_ptr) {
    std::vector<double> weights;
    weights.push_back((1.0 - remainder.x()) * (1.0 - remainder.y()));
    weights.push_back(remainder.x() * (1.0 - remainder.y()));
    weights.push_back((1.0 - remainder.x()) * remainder.y());
    weights.push_back(remainder.x() * remainder.y());

    for (double weight : weights) {
      T typed_weight;
      if (std::is_floating_point<T>()) {
        typed_weight = weight;
      } else {
        // convert to fixed point notation of same size as type
        typed_weight = static_cast<T>(
            static_cast<double>((std::numeric_limits<T>::max() - 1) / 2) *
            weight);
      }

      cl_uchar* byte_ptr = reinterpret_cast<cl_uchar*>(&typed_weight);
      for (size_t i = 0; i < sizeof(T); ++i) {
        weight_map_ptr->push_back(byte_ptr[i]);
      }
    }
  }

  static void stringSub(const std::string& string_to_find,
                        const std::string& string_to_sub_in,
                        std::string* string_ptr);

  cl::Program::Sources createSources(const KernelSourceInfo& source_info);
  cl::Device setupDevice();

  std::shared_ptr<KernelSourceInfo> source_info_ptr_;

  std::vector<cl_uint> idx_map_;
  cl::Buffer idx_buffer_;

  std::vector<cl_uchar> weight_map_;
  cl::Buffer weight_buffer_;

  cl::Device device_;
  cl::Context context_;
  cl::Program program_;
  cl::CommandQueue queue_;

#endif
};
}
#endif