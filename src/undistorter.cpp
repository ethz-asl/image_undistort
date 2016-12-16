#include "image_undistort/undistorter.h"

namespace image_undistort {
Undistorter::Undistorter(
    const CameraParametersPair& input_camera_parameters_pair)
    : used_camera_parameters_pair_(input_camera_parameters_pair) {
  if (!used_camera_parameters_pair_.valid()) {
    throw std::runtime_error(
        "Attempted to create undistorter from invalid camera parameters");
  }
  const cv::Size& resolution_out =
      used_camera_parameters_pair_.getOutputPtr()->resolution();
  const cv::Size& resolution_in =
      used_camera_parameters_pair_.getInputPtr()->resolution();
  // Initialize maps
  cv::Mat map_x_float(resolution_out, CV_32FC1);
  cv::Mat map_y_float(resolution_out, CV_32FC1);

  std::vector<double> D;
  if (used_camera_parameters_pair_.distortionProcessing() ==
      DistortionProcessing::UNDISTORT) {
    D = used_camera_parameters_pair_.getInputPtr()->D();
  } else {
    D = std::vector<double>(0, 5);
  }

  empty_pixels_ = false;

  // Compute the remap maps
  for (size_t v = 0; v < resolution_out.height; ++v) {
    for (size_t u = 0; u < resolution_out.width; ++u) {
      Eigen::Vector2d pixel_location(u, v);
      Eigen::Vector2d distorted_pixel_location;
      distortPixel(
          used_camera_parameters_pair_.getInputPtr()->P(),
          used_camera_parameters_pair_.getOutputPtr()->P(),
          used_camera_parameters_pair_.getInputPtr()->distortionModel(), D,
          pixel_location, &distorted_pixel_location);

      // Insert in map
      map_x_float.at<float>(v, u) =
          static_cast<float>(distorted_pixel_location.x());
      map_y_float.at<float>(v, u) =
          static_cast<float>(distorted_pixel_location.y());

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
}

void Undistorter::undistortImage(const cv::Mat& image,
                                 cv::Mat* undistorted_image) {
  if (empty_pixels_) {
    cv::remap(image, *undistorted_image, map_x_, map_y_, cv::INTER_LINEAR,
              cv::BORDER_CONSTANT);
  } else {
    // replicate is more efficient for gpus to calculate
    cv::remap(image, *undistorted_image, map_x_, map_y_, cv::INTER_LINEAR,
              cv::BORDER_REPLICATE);
  }
}

const CameraParametersPair& Undistorter::getCameraParametersPair() {
  return used_camera_parameters_pair_;
}

void Undistorter::distortPixel(const Eigen::Matrix<double, 3, 4>& P_in,
                               const Eigen::Matrix<double, 3, 4>& P_out,
                               const DistortionModel& distortion_model,
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

  switch(distortion_model){
  case DistortionModel::RADTAN: {
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
  } break;
  case DistortionModel::EQUIDISTANT: {
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
  } break;
  case DistortionModel::FOV: {
    // Split out parameters for easier reading
    const double &fov = D[0];

    const double r = std::sqrt(x * x + y * y);
    if (r < 1e-10) {
      *distorted_pixel_location = pixel_location;
      return;
    }
    double rd = (1.0 / fov) * atan(2.0 * tan(fov / 2.0) * r);
    xd = x * (rd / r);
    yd = y * (rd / r);
  } break;
  default:
    throw std::runtime_error("Distortion model not implemented - model: " +
                             static_cast<int>(distortion_model));
  }

  *distorted_pixel_location =
      P_in.topLeftCorner<2, 3>() * norm_distorted_pixel_location;
}
}