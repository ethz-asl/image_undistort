#include "image_undistort/interpolator.h"

namespace image_undistort {

Interpolator::Interpolator(const cv::Size& input_image_size,
                           const DistortionMap& distortion_map)
    : device_(setupDevice()),
      context_(device_),
      program_(context_, createSources()),
      queue_(context_, device_),
      output_image_size_(distortion_map.cols(), distortion_map.rows()) {
  if (program_.build({device_}) != CL_SUCCESS) {
    ROS_ERROR_STREAM("Error building: "
                     << program_.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device_));
    throw std::runtime_error("Error building OpenCL kernel");
  }

  // images are row major
  for (size_t j = 0; j < distortion_map.rows(); ++j) {
    for (size_t i = 0; i < distortion_map.cols(); ++i) {
      // nearest neighbour
      // idx_map_.push_back(std::round(distortion_map(j, i).x()) +
      //                   std::round(distortion_map(j, i).y()) *
      //                       input_image_size.width);

      const Eigen::Vector2d& point = distortion_map(j, i);
      Eigen::Vector2i point_int(std::round(point.x()), std::round(point.y()));
      Eigen::Vector2d point_rem(point.x() - point_int.x(),
                                point.y() - point_int.y());

      idx_map_.push_back(point_int.x() +
                         point_int.y() * input_image_size.width);

      // 8 possible interpolation directions

      // top left
      if ((point_rem.x() <= -0.25) && (point_rem.y()) <= -0.25) {
        offset_map_.push_back(-input_image_size.width - 1);
        weight_map_.push_back((std::sqrt(point_rem.x() * point_rem.x() +
                                         point_rem.y() * point_rem.y()) /
                               std::sqrt(2.0)) *
                              128);
      }
      // top
      else if ((point_rem.x() < 0.25) && (point_rem.y()) <= -0.25) {
        offset_map_.push_back(-input_image_size.width);
        weight_map_.push_back(std::abs(point_rem.y()) * 128);
      }
      // top right
      else if ((point_rem.x() >= 0.25) && (point_rem.y()) <= -0.25) {
        offset_map_.push_back(-input_image_size.width + 1);
        weight_map_.push_back((std::sqrt(point_rem.x() * point_rem.x() +
                                         point_rem.y() * point_rem.y()) /
                               std::sqrt(2.0)) *
                              128);
      }
      // left
      else if ((point_rem.x() <= 0) && (point_rem.y()) < 0.25) {
        offset_map_.push_back(-1);
        weight_map_.push_back(std::abs(point_rem.x()) * 128);
      }
      // right
      else if ((point_rem.x() > 0) && (point_rem.y()) < 0.25) {
        offset_map_.push_back(1);
        weight_map_.push_back(std::abs(point_rem.x()) * 128);
      }
      // bottom left
      else if ((point_rem.x() <= -0.25) && (point_rem.y()) >= 0.25) {
        offset_map_.push_back(input_image_size.width - 1);
        weight_map_.push_back((std::sqrt(point_rem.x() * point_rem.x() +
                                         point_rem.y() * point_rem.y()) /
                               std::sqrt(2.0)) *
                              128);
      }
      // bottom
      else if ((point_rem.x() < 0.25) && (point_rem.y()) >= 0.25) {
        offset_map_.push_back(input_image_size.width);
        weight_map_.push_back(std::abs(point_rem.y()) * 128);
      }
      // bottom right
      else if ((point_rem.x() >= 0.25) && (point_rem.y()) >= 0.25) {
        offset_map_.push_back(input_image_size.width + 1);
        weight_map_.push_back((std::sqrt(point_rem.x() * point_rem.x() +
                                         point_rem.y() * point_rem.y()) /
                               std::sqrt(2.0)) *
                              128);
      } else {
        throw std::runtime_error("invalid interpolation location");
      }
    }
  }

  idx_buffer_ = cl::Buffer(context_, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY,
                           sizeof(cl_uint) * idx_map_.size(), idx_map_.data());
  offset_buffer_ =
      cl::Buffer(context_, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY,
                 sizeof(cl_short) * offset_map_.size(), offset_map_.data());
  weight_buffer_ =
      cl::Buffer(context_, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY,
                 sizeof(cl_uchar) * weight_map_.size(), weight_map_.data());
};

void Interpolator::Interpolate(const cv::Mat& image,
                               cv::Mat* undistorted_image) {
  *undistorted_image = cv::Mat(output_image_size_, image.type());

  cl::Buffer input_image_buffer(context_,
                                CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY,
                                image.elemSize() * image.total(), image.data);
  cl::Buffer output_image_buffer(
      context_, CL_MEM_USE_HOST_PTR | CL_MEM_WRITE_ONLY,
      undistorted_image->elemSize() * undistorted_image->total(),
      undistorted_image->data);

  cl::Kernel kernel_inter = cl::Kernel(program_, "interpolate");
  kernel_inter.setArg(0, input_image_buffer);
  kernel_inter.setArg(1, idx_buffer_);
  //kernel_inter.setArg(2, offset_buffer_);
  //kernel_inter.setArg(3, weight_buffer_);
  kernel_inter.setArg(2, output_image_buffer);
  queue_.enqueueNDRangeKernel(
      kernel_inter, cl::NullRange,
      cl::NDRange(output_image_size_.height * output_image_size_.width),
      cl::NullRange);
  queue_.finish();

  // hopefully forces the output image back into host memory
  queue_.enqueueMapBuffer(output_image_buffer, CL_TRUE, CL_MAP_READ, 0,
                          output_image_size_.height * output_image_size_.width);
}

cl::Device Interpolator::setupDevice() {
  // get all platforms (drivers)
  std::vector<cl::Platform> all_platforms;
  cl::Platform::get(&all_platforms);
  if (all_platforms.size() == 0) {
    throw std::runtime_error("No platforms found. Check OpenCL installation!");
    exit(1);
  }
  cl::Platform default_platform = all_platforms[0];
  ROS_INFO_STREAM("Using OpenCL platform: "
                  << default_platform.getInfo<CL_PLATFORM_NAME>());

  // get default device of the default platform
  std::vector<cl::Device> all_devices;
  default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
  if (all_devices.size() == 0) {
    throw std::runtime_error("No devices found. Check OpenCL installation!");
  }
  cl::Device default_device = all_devices[0];
  ROS_INFO_STREAM(
      "Using OpenCL device: " << default_device.getInfo<CL_DEVICE_NAME>());

  return default_device;
}

cl::Program::Sources Interpolator::createSources() {
  cl::Program::Sources sources;

  std::string kernel_code =
      "void kernel interpolate(global const uchar* data_in,"
      "                              global const uint* idx,"
      "                              global uchar* data_out) {"
      "  data_out[get_global_id(0)] = data_in[idx[get_global_id(0)]];"
      "}";
  sources.push_back({kernel_code.c_str(), kernel_code.length()});
/*
  std::string kernel_code =
      "void kernel interpolate(global const uchar* data_in,"
      "                              global const uint* idx,"
      "                              global const short* offset,"
      "                              global const uchar* weight,"
      "                              global uchar* data_out) {"
      "  const uint pixel_idx = idx[get_global_id(0)];"
      "  const short pixel_offset = offset[get_global_id(0)];"
      "  const uchar pixel_weight = weight[get_global_id(0)];"
      "  const ushort a = data_in[pixel_idx];"
      "  short b = data_in[pixel_idx + pixel_offset];"
      "  b = b - a;"
      "  short c = data_in[pixel_idx - pixel_offset];"
      "  c = a - c;"
      "  short res = min(abs(b),abs(c));"
      "  res = b - 2*((b < 0)&b);"
      "  const ushort val16 = 128*a + pixel_weight * res;"
      "  const uchar val8 = val16 >> 7;"
      "  data_out[get_global_id(0)] = val8;"
      "}";
  sources.push_back({kernel_code.c_str(), kernel_code.length()});*/

  return sources;
}
}