#include <image_undistort/undistorter.h>

namespace image_undistort {
Undistorter::Undistorter(
    const CameraParametersPair& input_camera_parameters_pair)
    : used_camera_parameters_pair_(input_camera_parameters_pair) {
  if (!used_camera_parameters_pair_.valid()) {
    throw std::runtime_error(
        "Attempted to create undistorter from invalid camera parameters");
  }

  // setup opencl stuff

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

  cl_context_ = std::make_shared<cl::Context>(default_device);

  cl::Program::Sources sources;

  // note a uint8 is not the same as a uint8_t, use unsigned char instead
  /*std::string kernel_code =
      "   void kernel nearest_neighbour(global const unsigned char* data_in, "
      "global const int* idx, global unsigned char* data_out){       "
      "       data_out[get_global_id(0)] = data_in[idx[get_global_id(0)]];     "
      "        "
      "   }                                                                    "
      "           ";*/

  std::string kernel_code =
      "void kernel linear_inter(global const unsigned char* data_in, global "
      "const int* const idx1, global const float* const w1, global const int* "
      "const idx2, global const float* const w2, global unsigned char* const "
      "data_out){"
      "  data_out[get_global_id(0)] = "
      "w1[get_global_id(0)]*data_in[idx1[get_global_id(0)]] + "
      "w2[get_global_id(0)]*data_in[idx2[get_global_id(0)]];"
      "}";
  sources.push_back({kernel_code.c_str(), kernel_code.length()});

  cl_program_ = std::make_shared<cl::Program>(*cl_context_, sources);
  if (cl_program_->build({default_device}) != CL_SUCCESS) {
    ROS_ERROR_STREAM(
        "Error building: " << cl_program_->getBuildInfo<CL_PROGRAM_BUILD_LOG>(
            default_device));
    throw std::runtime_error("Error building OpenCL kernel");
  }

  // create queue to which we will push commands for the device.
  cl_queue_ = std::make_shared<cl::CommandQueue>(*cl_context_, default_device);

  const cv::Size& resolution_out =
      used_camera_parameters_pair_.getOutputPtr()->resolution();
  const cv::Size& resolution_in =
      used_camera_parameters_pair_.getInputPtr()->resolution();
  // Initialize maps
  cv::Mat map_x_float(resolution_out, CV_32FC1);
  cv::Mat map_y_float(resolution_out, CV_32FC1);

  std::vector<double> D;
  if (used_camera_parameters_pair_.undistort()) {
    D = used_camera_parameters_pair_.getInputPtr()->D();
  } else {
    D = std::vector<double>(0, 5);
  }

  empty_pixels_ = false;

  // create local host memory maps
  std::vector<int> map_e1_host, map_e2_host;
  std::vector<float> map_f1_host, map_f2_host;

  // Compute the remap maps
  for (size_t v = 0; v < resolution_out.height; ++v) {
    for (size_t u = 0; u < resolution_out.width; ++u) {
      Eigen::Vector2d pixel_location(u, v);
      Eigen::Vector2d distorted_pixel_location;
      distortPixel(
          used_camera_parameters_pair_.getInputPtr()->P(),
          used_camera_parameters_pair_.getOutputPtr()->P(),
          used_camera_parameters_pair_.getInputPtr()->usingRadtanDistortion(),
          D, pixel_location, &distorted_pixel_location);

      // Insert in map
      map_x_float.at<float>(v, u) =
          static_cast<float>(distorted_pixel_location.x());
      map_y_float.at<float>(v, u) =
          static_cast<float>(distorted_pixel_location.y());

      double xd = distorted_pixel_location.x();
      double yd = distorted_pixel_location.y();
      int xi1 = std::round(xd);
      int yi1 = std::round(yd);

      int xi2;
      if (std::abs(xd - xi1) < 0.25) {
        xi2 = xi1;
      } else {
        xi2 = (xd > xi1 ? 1 : -1) + xi1;
      }
      int yi2;
      if (std::abs(yd - yi1) < 0.25) {
        yi2 = yi1;
      } else {
        yi2 = (yd > yi1 ? 1 : -1) + yi1;
      }

      float f1 = std::sqrt((xi1 - xd) * (xi1 - xd) + (yi1 - yd) * (yi1 - yd));
      float f2 = std::sqrt((xi2 - xd) * (xi2 - xd) + (yi2 - yd) * (yi2 - yd));
      float ft = f1 + f2;
      f1 /= ft;
      f2 /= ft;

      map_e1_host.push_back(xi1 + resolution_in.width * yi1);
      map_e2_host.push_back(xi2 + resolution_in.width * yi2);
      map_f1_host.push_back(f1);
      map_f2_host.push_back(f2);

      if ((distorted_pixel_location.x() < 0) ||
          (distorted_pixel_location.y() < 0) ||
          (distorted_pixel_location.x() >= resolution_in.width) ||
          (distorted_pixel_location.y() >= resolution_in.height)) {
        empty_pixels_ = true;
      }
    }
  }

  // convert to fixed point maps for increased speed
  cv::convertMaps(map_x_float, map_y_float, map_x_, map_y_, CV_16SC2);

  // copy to device memory
  map_e1_ = std::make_shared<cl::Buffer>(*cl_context_, CL_MEM_READ_ONLY,
                                         sizeof(int) * map_e1_host.size());
  cl_queue_->enqueueWriteBuffer(*map_e1_, CL_TRUE, 0,
                                sizeof(int) * map_e1_host.size(),
                                map_e1_host.data());
  map_e2_ = std::make_shared<cl::Buffer>(*cl_context_, CL_MEM_READ_ONLY,
                                         sizeof(int) * map_e2_host.size());
  cl_queue_->enqueueWriteBuffer(*map_e2_, CL_TRUE, 0,
                                sizeof(int) * map_e2_host.size(),
                                map_e2_host.data());

  map_f1_ = std::make_shared<cl::Buffer>(*cl_context_, CL_MEM_READ_ONLY,
                                         sizeof(float) * map_f1_host.size());
  cl_queue_->enqueueWriteBuffer(*map_f1_, CL_TRUE, 0,
                                sizeof(float) * map_f1_host.size(),
                                map_f1_host.data());
  map_f2_ = std::make_shared<cl::Buffer>(*cl_context_, CL_MEM_READ_ONLY,
                                         sizeof(float) * map_f2_host.size());
  cl_queue_->enqueueWriteBuffer(*map_f2_, CL_TRUE, 0,
                                sizeof(float) * map_f2_host.size(),
                                map_f2_host.data());
}

void Undistorter::undistortImage(const cv::Mat& image,
                                 cv::Mat* undistorted_image) {
  undistorted_image->create(map_x_.rows, map_x_.cols, image.type());
  cl::Buffer data_in(*cl_context_, CL_MEM_USE_HOST_PTR,
                     sizeof(uint8_t) * image.total(), image.data);
  cl::Buffer data_out(*cl_context_, CL_MEM_USE_HOST_PTR,
                      sizeof(uint8_t) * undistorted_image->total(),
                      undistorted_image->data);

  // alternative way to run the kernel
  cl::Kernel kernel_linear_inter = cl::Kernel(*cl_program_, "linear_inter");
  kernel_linear_inter.setArg(0, data_in);
  kernel_linear_inter.setArg(1, *map_e1_);
  kernel_linear_inter.setArg(2, *map_f1_);
  kernel_linear_inter.setArg(3, *map_e2_);
  kernel_linear_inter.setArg(4, *map_f2_);
  kernel_linear_inter.setArg(5, data_out);
  cl_queue_->enqueueNDRangeKernel(kernel_linear_inter, cl::NullRange,
                                  cl::NDRange(undistorted_image->total()),
                                  cl::NullRange);
  cl_queue_->finish();

  cl_queue_->enqueueMapBuffer(data_out, CL_TRUE, CL_MAP_READ, 0,
                              sizeof(uint8_t) * undistorted_image->total());

 //  cv::remap(image, *undistorted_image, map_x_, map_y_, cv::INTER_LINEAR,
 //           cv::BORDER_REPLICATE);
  
}

const CameraParametersPair& Undistorter::getCameraParametersPair() {
  return used_camera_parameters_pair_;
}

void Undistorter::distortPixel(const Eigen::Matrix<double, 3, 4>& P_in,
                               const Eigen::Matrix<double, 3, 4>& P_out,
                               const bool using_radtan,
                               const std::vector<double>& D,
                               const Eigen::Vector2d& pixel_location,
                               Eigen::Vector2d* distorted_pixel_location) {
  // Transform image coordinates to be size and focus independent
  Eigen::Vector2d norm_pixel_location =
      P_out.topLeftCorner<2, 2>().inverse() *
      (pixel_location - P_out.block<2, 1>(0, 2));

  const double& x = norm_pixel_location.x();
  const double& y = norm_pixel_location.y();

  Eigen::Vector3d norm_distorted_pixel_location(0, 0, 1);
  double& xd = norm_distorted_pixel_location.x();
  double& yd = norm_distorted_pixel_location.y();

  if (using_radtan) {
    // Split out parameters for easier reading
    const double& k1 = D[0];
    const double& k2 = D[1];
    const double& k3 = D[4];
    const double& p1 = D[2];
    const double& p2 = D[3];

    // Undistort
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double kr = (1.0 + k1 * r2 + k2 * r4 + k3 * r6);
    xd = x * kr + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    yd = y * kr + 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y);

  } else {
    // Split out distortion parameters for easier reading
    const double& k1 = D[0];
    const double& k2 = D[1];
    const double& k3 = D[2];
    const double& k4 = D[3];

    // Undistort
    const double r = std::sqrt(x * x + y * y);
    if (r < 1e-10) {
      *distorted_pixel_location = pixel_location;
      return;
    }
    const double theta = atan(r);
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta2 * theta4;
    const double theta8 = theta4 * theta4;
    const double thetad =
        theta * (1 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);

    const double scaling = (r > 1e-8) ? thetad / r : 1.0;
    xd = x * scaling;
    yd = y * scaling;
  }

  *distorted_pixel_location =
      P_in.topLeftCorner<2, 3>() * norm_distorted_pixel_location;
}
}