#include <image_undistort/undistorter.h>

Undistorter::Undistorter(const Eigen::Matrix3d& K, const std::vector<double>& D,
                         const cv::Size& resolution, const bool using_radtan, const double zoom)
    : using_radtan_(using_radtan) {
  // Initialize maps
  map_x_.create(resolution, CV_32FC1);
  map_y_.create(resolution, CV_32FC1);

  // Compute the remap maps
  for (size_t v = 0; v < resolution.height; ++v) {
    for (size_t u = 0; u < resolution.width; ++u) {
      double ud, vd;
      distortPixel(K, D, zoom, u, v, &ud, &vd);

      // Insert in map
      map_x_.at<float>(v, u) = ud;
      map_y_.at<float>(v, u) = vd;
    };
  }
}

void Undistorter::undistortImage(const cv::Mat& image,
                                 cv::Mat* undistorted_image) {
  cv::remap(image, *undistorted_image, map_x_, map_y_, CV_INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
}

void Undistorter::distortPixel(const Eigen::Matrix3d& K,
                               const std::vector<double>& D, const double zoom, const double u,
                               const double v, double* ud, double* vd) {
  // Transform image coordinates to be size and focus independent
  double x = (u - K(0, 2)) / K(0, 0);
  double y = (v - K(1, 2)) / K(1, 1);

  x *= 1.0/zoom;
  y *= 1.0/zoom;

  double xd, yd;

  if (using_radtan_) {
    // Split out distortion parameters for easier reading
    double k1 = D[0];
    double k2 = D[1];
    double k3 = D[4];
    double p1 = D[2];
    double p2 = D[3];

    // Undistort
    double r2 = x * x + y * y;
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    double kr = (1.0 + k1 * r2 + k2 * r4 + k3 * r6);
    xd = x * kr + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    yd = y * kr + 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y);

  } else {
    // Split out distortion parameters for easier reading
    double k1 = D[0];
    double k2 = D[1];
    double k3 = D[2];
    double k4 = D[3];

    // Undistort
    double r = std::sqrt(x * x + y * y);
    if (r < 1e-10) {
      *ud = u;
      *vd = v;
      return;
    }
    const double theta = atan(r);
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta2 * theta4;
    const double theta8 = theta4 * theta4;
    const double thetad =
        theta * (1 + k1*theta2 + k2*theta4 + k3*theta6 + k4*theta8);

    const double scaling = (r > 1e-8) ? thetad / r : 1.0;
    xd = x * scaling;
    yd = y * scaling;
  }

  // Shift and scale back
  *ud = K(0, 0) * xd + K(0, 2);
  *vd = K(1, 1) * yd + K(1, 2);
};