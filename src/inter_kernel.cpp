
// setup opencl stuff
InterKernel::InterKernel():
device_(setupDevice()),
context_(device_),
program_(context_, createSources()),
queue_(context_, device_){};

    cl::Device InterKernel::setupDevice() {
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

cl::Program::Sources InterKernel::createSources() {
  cl::Program::Sources sources;

  std::string kernel_code = 
  "void kernel nearest_neighbour(global const IMAGE_TYPE* data_in,"
  "                              global const int* idx,"
  "                              global IMAGE_TYPE* data_out) {"
  "  data_out[get_global_id(0)] = data_in[idx[get_global_id(0)]];"
  "}";
  sources.push_back({kernel_code.c_str(), kernel_code.length()});

  kernel_code = 
  "void kernel linear_inter("
  "    global const IMAGE_TYPE* data_in, global const int* const idx1,"
  "    global const float* const w1, global const int* const idx2,"
  "    global const float* const w2, global IMAGE_TYPE* const data_out) {"
  "  data_out[get_global_id(0)] ="
  "      w1[get_global_id(0)] * data_in[idx1[get_global_id(0)]] +"
  "      w2[get_global_id(0)] * data_in[idx2[get_global_id(0)]]";
  "}";
  sources.push_back({kernel_code.c_str(), kernel_code.length()});

  return sources;
}

  cl_program_ = std::make_shared<cl::Program>(*cl_context_, sources);
  if (cl_program_->build({default_device}) != CL_SUCCESS) {
    ROS_ERROR_STREAM(
        "Error building: " << cl_program_->getBuildInfo<CL_PROGRAM_BUILD_LOG>(
            default_device));
    throw std::runtime_error("Error building OpenCL kernel");
  }

  // create queue to which we will push commands for the device.
  cl_queue_ = std::make_shared<cl::CommandQueue>(*cl_context_, default_device);