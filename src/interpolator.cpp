#include "image_undistort/interpolator.h"

namespace image_undistort {

KernelSourceInfo::KernelSourceInfo(const cv::Size& input_image_size,
                                   const int image_type,
                                   const size_t number_channels,
                                   const bool empty_pixels)
    : input_image_size_(input_image_size),
      image_type_(image_type),
      number_channels_(number_channels),
      empty_pixels_(empty_pixels) {}

Interpolator::Interpolator(const KernelSourceInfo& source_info,
                           const DistortionMap& distortion_map)
    : device_(setupDevice()),
      context_(device_),
      program_(context_, createSources(source_info)),
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

      Eigen::Vector2d pxel_location = distortion_map(v, u);
      Eigen::Vector2i top_left(std::floor(pixel_location.x()),
                               std::floor(pixel_location.y()));
      Eigen::Vector2d remainder = pixel_location - top_left.cast<double>();

      if ((top_left.x() < 0) || (top_left.y() < 0) ||
          ((top_left.x() + 1) >= distortion_map.cols()) ||
          ((top_left.y() + 1) >= distortion_map.rows())) {
        idx_map_.push_back(static_cast<cl_uint>(NULL));
      } else {
        idx_map_.push_back(top_left.x() + source_info.width * top_left.y());
      }

      switch (source_info.image_type_) {
        case CV_8U:
        case CV_8S:
          addWeight<cl_uchar>(remainder, &weight_map_);
          break;
        case CV_16U:
        case CV_16S:
          addWeight<cl_ushort>(remainder, &weight_map_);
          break;
        case CV_32S:
          addWeight<cl_uint>(remainder, &weight_map_);
          break;
        case CV_32F:
          addWeight<float>(remainder, &weight_map_);
          break;
        case CV_64F:
          addWeight<double>(remainder, &weight_map_);
          break;
        default:
          throw std::runtime_error("Invalid opencv image type");
      }
    }
  }

  idx_buffer_ = cl::Buffer(context_, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY,
                           sizeof(cl_uint) * idx_map_.size(), idx_map_.data());

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
  // kernel_inter.setArg(2, offset_buffer_);
  // kernel_inter.setArg(3, weight_buffer_);
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

void Interpolator::stringSub(const std::string& string_to_find,
                             const std::string& string_to_sub_in,
                             std::string* string_ptr) {
  for (std::string::size_type i = 0; i != std::string::npos;
       i = string_ptr->find(string_to_find, i)) {
    string_ptr->replace(i, string_to_find.size(), string_to_sub_in);
    i += string_to_sub_in.size();
  }
}

cl::Program::Sources Interpolator::createSources(
    const KernelSourceInfo& source_info) {
  cl::Program::Sources sources;

  // Warning: what follows is probably some of the ugliest code I have ever
  // written
  std::string kernel_code =
      "void kernel interpolate("
      "    global const _SIGNED__SMALL_TYPE__CHANNELS_* data_in,"
      "    global const uint* idxs, global const _SMALL_TYPE_4* weights,"
      "    global _SIGNED__SMALL_TYPE__CHANNELS_* data_out) {"
      "  const uint offset = get_global_id(0);"
      "  const uint idx = idxs[offset];"
      "";

  // only do out of bounds checking if we absolutely need to
  if (source_info.empty_pixels_) {
    kernel_code +=
        "  if (idx == NULL) {"
        "    data_out[offset] = 0;"
        "    return;"
        "  }"
        "";
  }

  kernel_code +=
      "const _SMALL_TYPE_4 weight = weights[offset];"
      ""
      "_SIGNED__BIG_TYPE__CHANNELS_ pixel ="
      "    convert__SIGNED__BIG_TYPE_(weight.s0()) *"
      "    convert__SIGNED__BIG_TYPE__CHANNELS_(data_in[idx]);"
      "pixel += convert__SIGNED__BIG_TYPE_(weight.s1()) *"
      "         convert__SIGNED__BIG_TYPE__CHANNELS_(data_in[idx + 1]);"
      "pixel += convert__SIGNED__BIG_TYPE_(weight.s2()) *"
      "         convert__SIGNED__BIG_TYPE__CHANNELS_(data_in[idx + "
      "_Y_OFFSET_]);"
      "pixel +="
      "    convert__SIGNED__BIG_TYPE_(weight.s3()) *"
      "    convert__SIGNED__BIG_TYPE__CHANNELS_(data_in[idx + _Y_OFFSET_ + 1]);"
      "";

  // shifting is only needed if we are using fixed point weights
  if ((source_info.image_type_ != CV_32F) ||
      (source_info.image_type != CV_64F)) {
    kernel_code +=
        "data_out[offset] = convert__SIGNED__SMALL_TYPE_(pixel >> _SHIFT_);";
  }

  kernel_code += "}";

  // swap in hard-coded values (super ugly but efficient hack that beats writing
  // 28 subtly different kernels)
  //_SMALL_TYPE_: type of raw data
  //_BIG_TYPE_: type large enough to store _SMALL_TYPE_*_SMALL_TYPE_ (only
  //  needed for integer types when fixed point weights are used, floating
  //  point can have _SMALL_TYPE_ = _BIG_TYPE_)
  //_SIGNED_: indicates if the type is signed
  //_CHANNELS_: number of image channels (can be 1, 2, 3, 4, 8 or 16)
  //_SHIFT_: amount to shift data back by when converting from _BIG_TYPE_ to
  //  _SMALL_TYPE_
  //_YOFFSET_: amount that must be added to an index to shift by 1 row

  stringSub("_Y_OFFSET_", std::to_string(source_info.input_image_size_.width),
            &kernel_code);

  if (source_info.number_channels_ == 1) {
    stringSub("_CHANNELS_", "", &kernel_code);
  } else {
    stringSub("_CHANNELS_", std::to_string(number_channels), &kernel_code);
  }

  switch (source_info.image_type_) {
    case CV_8U:
      stringSub("_SMALL_TYPE_", "uchar", &kernel_code);
      stringSub("_BIG_TYPE_", "ushort", &kernel_code);
      stringSub("_SHIFT_", "8", &kernel_code);
      stringSub("_SIGNED_", "u", &kernel_code);
      break;
    case CV_8S:
      stringSub("_SMALL_TYPE_", "uchar", &kernel_code);
      stringSub("_BIG_TYPE_", "ushort", &kernel_code);
      stringSub("_SHIFT_", "8", &kernel_code);
      stringSub("_SIGNED_", "", &kernel_code);
      break;
    case CV_16U:
      stringSub("_SMALL_TYPE_", "ushort", &kernel_code);
      stringSub("_BIG_TYPE_", "uint", &kernel_code);
      stringSub("_SHIFT_", "16", &kernel_code);
      stringSub("_SIGNED_", "u", &kernel_code);
      break;
    case CV_16S:
      stringSub("_SMALL_TYPE_", "short", &kernel_code);
      stringSub("_BIG_TYPE_", "int", &kernel_code);
      stringSub("_SHIFT_", "16", &kernel_code);
      stringSub("_SIGNED_", "", &kernel_code);
      break;
    case CV_32S:
      stringSub("_SMALL_TYPE_", "short", &kernel_code);
      stringSub("_BIG_TYPE_", "int", &kernel_code);
      stringSub("_SHIFT_", "32", &kernel_code);
      stringSub("_SIGNED_", "", &kernel_code);
      break;
    case CV_32F:
      stringSub("_SMALL_TYPE_", "float", &kernel_code);
      stringSub("_BIG_TYPE_", "float", &kernel_code);
      stringSub("_SIGNED_", "", &kernel_code);
      break;
    case CV_64F:
      stringSub("_SMALL_TYPE_", "double", &kernel_code);
      stringSub("_BIG_TYPE_", "double", &kernel_code);
      stringSub("_SIGNED_", "", &kernel_code);
      break;
    default:
      throw std::runtime_error("Invalid opencv image type");
  }
  sources.push_back({kernel_code.c_str(), kernel_code.length()});

  return sources;
}
}