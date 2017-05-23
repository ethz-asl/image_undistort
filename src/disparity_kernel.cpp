#include "image_undistort/disparity_kernel.h"

namespace image_undistort {

DisparityKernel::DisparityKernel()
  : source_info_ptr_(nullptr),
    device_(setupDevice()),
    context_(device_),
    queue_(context_, device_) {

  program_ = cl::Program(context_, createSources(source_info));
  if (program_.build({device_}) != CL_SUCCESS) {
    throw std::runtime_error(
        "Error building: " +
        program_.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device_));
  }
}

void DisparityKernel::computeDisparity(const cv::Mat& left_image, const cv::Mat& right_image, cv::Mat* disparity_image){

  if((left_image.type() != CV_8UC1) || (right_image.type() != CV_8UC1)){
    throw std::runtime_error("input images must have type CV_8UC1")
  }
  if((left_image.cols != right_image.cols) || (left_image.rows != right_image.rows)){
    throw std::runtime_error("input images must be the same size")
  }
  if(disparity_image == nullptr){
    throw std::runtime_error("invalid disparity image pointer");
  }

  cl::Buffer left_image_buffer(context_,
                                CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY,
                                left_image.elemSize() * left_image.total(), left_image.data);
  cl::Buffer right_image_buffer(context_,
                              CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY,
                              right_image.elemSize() * right_image.total(), right_image.data);

  *disparity_image = Mat(left_image.rows, left_image.cols, CV_32S);
  cl::Buffer disparity_image_buffer(
      context_, CL_MEM_USE_HOST_PTR | CL_MEM_WRITE_ONLY,
      disparity_image->elemSize() * disparity_image->total(),
      disparity_image->data);


  cl::Kernel kernel_difference = cl::Kernel(program_, "difference");
  kernel_difference.setArg(0, left_image_buffer);
  kernel_difference.setArg(1, right_image_buffer);
  kernel_difference.setArg(2, disparity_image_buffer);
  queue_.enqueueNDRangeKernel(kernel_difference, cl::NullRange,
                              cl::NDRange(undistorted_image->total()),
                              cl::NullRange);
  queue_.finish();

  // hopefully forces the output image back cl_into host memory
  queue_.enqueueMapBuffer(
      disparity_image_buffer, CL_TRUE, CL_MAP_READ, 0,
      disparity_image->elemSize() * disparity_image->total());
}

cl::Device DisparityKernel::setupDevice() {
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

cl::Program::Sources DisparityKernel::createSources(
    const KernelSourceInfo& source_info) {
  cl::Program::Sources sources;

  // vload and vstore are missing for non-vector data, so we create them here
  std::string kernel_code =
    "void difference(global const cl_uchar* image_A, global const cl_uchar* image_B, global cl_int* diff_out){"
      "const cl_uint idx = get_global_id(0);"
      "diff_out = abs(convert_cl_int(image_A[idx]) - convert_cl_int(image_B[idx]));"
    "}";

  sources.push_back({kernel_code.c_str(), kernel_code.length()});

  return sources;
}

/*
void difference(global const cl_uchar* image_A, global const cl_uchar* image_B, global cl_int* diff_out){
  const cl_uint idx = get_global_id(0);
  diff_out = abs(convert_cl_int(image_A[idx]) - convert_cl_int(image_B[idx]));
}
/*
void colIntegrate(global const cl_int* image_in, global const cl_int rows, global const cl_int cols, global cl_int* image_out){
  cl_uint offset = get_global_id(0);
  image_out[offset] = image_in[offset];

  for(cl_uint i = offset; i < rows*(cols-1); i += cols){
    image_out[i + cols] = image_in[i + cols] + image_out[i];
  }
}

void rowIntegrate(const cl_int* image_in, const cl_int rows, const cl_int cols, cl_int* image_out){
  cl_uint offset = cols*get_global_id(0);
  image_out[offset] = image_in[offset];

  for(cl_uint i = offset; i < (offset + rows - 1); ++i){
    image_out[i + 1] = image_in[i + 1] + image_out[i];
  }
}

void blockError(const cl_int* cl_int_image, const cl_int block_size, const cl_int rows, const cl_int cols, cl_int* error_out){
  const cl_int idx = get_global_id(0);
  const cl_int offset = block_size + block_size*cols;

  const cl_int idx_top_left = max(0, idx - offset);
  const cl_int idx_bottom_right = min(cols*rows-1, idx + offset);

  error_out[idx] = cl_int_image[idx_bottom_right] - cl_int_image[idx_top_left];
}

void invalidateLeftAndRight(int* disp_image, const cl_int rows, const cl_int cols){
  const cl_uint idx = get_global_id(0);
  disp_image[idx] = -1;
  disp_image[rows*cols-idx-1] = -1;
}

void invalidateTopAndBottom(int* disp_image, const cl_int rows, const cl_int cols){
  const cl_uint idx = rows*get_global_id(0);
  disp_image[idx] = -1;
  disp_image[rows*cols-idx-1] = -1;
}

void initDisparity(const cl_int* current_error, const cl_int disparity_value, cl_int* disparity_image, cl_int* best_error){
  const cl_uint idx = get_global_id(0);
  disparity_image[idx] = disparity_value;
  best_error[idx] = current_error[idx];
}

void updateDisparity(const cl_int* current_error, const cl_int disparity_value, cl_int* disparity_image, cl_int* best_error){
  const cl_uint idx = get_global_id(0);
  if(current_error[idx] > best_error[idx]){
    disparity_image[idx] = disparity_value;
    best_error[idx] = current_error[idx];
  }
}*/