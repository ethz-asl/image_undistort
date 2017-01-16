#include <math.h>

#include "image_undistort/interpolator_tester.h"

namespace image_undistort {

void Tester::generateRotationMap(const cv::Size& image_size,
                         const double rotation_angle,
                         Interpolator::DistortionMap* distortion_map_ptr) {
  Eigen::Vector2d offset(static_cast<double>(image_size.width) / 2.0,
                         static_cast<double>(image_size.height) / 2.0);

  distortion_map_ptr->resize(image_size.height, image_size.width);

  // Compute the remap maps
  for (size_t v = 0; v < image_size.height; ++v) {
    for (size_t u = 0; u < image_size.width; ++u) {
      Eigen::Vector2d pixel_location(u, v);

      pixel_location -= offset;
      Eigen::Vector2d rotated_pixel_location(
          pixel_location.x() * std::cos(-rotation_angle) +
              pixel_location.y() * std::sin(-rotation_angle),
          pixel_location.x() * std::sin(-rotation_angle) +
              pixel_location.y() * std::cos(-rotation_angle));

      rotated_pixel_location += offset;

      // Insert in map
      (*distortion_map_ptr)(v, u).x() = rotated_pixel_location.x();
      (*distortion_map_ptr)(v, u).y() = rotated_pixel_location.y();
    }
  }
}

void Tester::generateRotationMap(const cv::Size& image_size,
                         const double rotation_angle, cv::Mat* map_x,
                         cv::Mat* map_y) {
  Interpolator::DistortionMap distortion_map;
  generateRotationMap(image_size, rotation_angle, &distortion_map);

  map_x->create(image_size, CV_32FC1);
  map_y->create(image_size, CV_32FC1);

  for (size_t v = 0; v < image_size.height; ++v) {
    for (size_t u = 0; u < image_size.width; ++u) {
      map_x->at<float>(v, u) = distortion_map(v, u).x();
      map_y->at<float>(v, u) = distortion_map(v, u).y();
    }
  }
}

Tester::Tester(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  // set parameters from ros
  std::string input_image_path, output_image_directory;
  int number_of_rotations;
  if (!nh_private_.getParam("input_image_path", input_image_path) ||
      !nh_private_.getParam("output_image_directory", output_image_directory) ||
      !nh_private_.getParam("number_of_rotations", number_of_rotations)) {
    ROS_ERROR("Missing input parameters, exiting");
    exit(0);
  }

  double rotation_angle = M_PI / static_cast<double>(number_of_rotations);

  cv::Mat input_image = cv::imread(input_image_path, CV_LOAD_IMAGE_GRAYSCALE);

  // convert to float to minimize discretization errors
  cv::Mat float_image;
  input_image.convertTo(float_image, CV_32FC1);

  // initialize images
  std::map<std::string, cv::Mat> images;
  images["nearest neighbor"] = float_image;
  images["bilinear"] = float_image;
  images["bicubic"] = float_image;
  images["lanczos4"] = float_image;

  // get remapping
  cv::Mat map_x, map_y;
  generateRotationMap(input_image.size(), rotation_angle, &map_x, &map_y);

  // rotate images
  for (size_t i = 0; i < number_of_rotations; ++i) {
    cv::Mat temp;

    cv::remap(images["nearest neighbor"], temp, map_x, map_y, INTER_NEAREST);
    images["nearest neighbor"] = temp;

    cv::remap(images["bilinear"], temp, map_x, map_y, INTER_LINEAR);
    images["bilinear"] = temp;

    cv::remap(images["bicubic"], temp, map_x, map_y, INTER_CUBIC);
    images["bicubic"] = temp;

    cv::remap(images["lanczos4"], temp, map_x, map_y, INTER_LANCZOS4);
    images["lanczos4"] = temp;
  }

  // lossless compression
  vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);

  // save results
  cv::Mat temp;

  images["nearest neighbor"].convertTo(temp, CV_8U1);
  imwrite(output_image_directory + '/nearest_neighbor.png', InputArray img,
          compression_params);

  images["bilinear"].convertTo(temp, CV_8U1);
  imwrite(output_image_directory + '/bilinear.png', InputArray img,
          compression_params);

  images["bicubic"].convertTo(temp, CV_8U1);
  imwrite(output_image_directory + '/bicubic.png', InputArray img,
          compression_params);

  images["lanczos4"].convertTo(temp, CV_8U1);
  imwrite(output_image_directory + '/lanczos4.png', InputArray img,
          compression_params);
}

}