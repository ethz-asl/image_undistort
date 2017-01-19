#include <math.h>

#include "image_undistort/interpolator_tester.h"

namespace image_undistort {

void Tester::interpolateNearestNeighbor(const cv::Mat input,
                                        const Interpolator::DistortionMap map,
                                        cv::Mat* output) {
  output->create(map.cols(), map.rows(), CV_32FC1);
  for (size_t v = 0; v < map.rows(); ++v) {
    for (size_t u = 0; u < map.cols(); ++u) {
      Eigen::Vector2d point = map(v, u);
      Eigen::Vector2i nn_point(std::round(point.x()), std::round(point.y()));

      if ((nn_point.x() < 0) || (nn_point.y() < 0) ||
          (nn_point.x() >= map.cols()) || (nn_point.y() >= map.rows())) {
        output->at<float>(v, u) = 0;
      } else {
        output->at<float>(v, u) = input.at<float>(nn_point.y(), nn_point.x());
      }
    }
  }
}

void Tester::interpolateNearestLinear(const cv::Mat input,
                                      const Interpolator::DistortionMap map,
                                      cv::Mat* output) {
  output->create(map.cols(), map.rows(), CV_32FC1);
  for (size_t v = 0; v < map.rows(); ++v) {
    for (size_t u = 0; u < map.cols(); ++u) {
      Eigen::Vector2d point = map(v, u);
      Eigen::Vector2i nn_point(std::round(point.x()), std::round(point.y()));
      Eigen::Vector2d rem_point = point - nn_point.cast<double>();

      Eigen::Vector2i current_point;
      Eigen::Vector2i opposite_point;
      double weight;
      if ((std::abs(rem_point.x()) > 0.25) ||
          (std::abs(rem_point.y()) > 0.25)) {
        current_point =
            nn_point + Eigen::Vector2i(std::copysign(1, rem_point.x()),
                                       std::copysign(1, rem_point.y()));
        opposite_point =
            nn_point - Eigen::Vector2i(std::copysign(1, rem_point.x()),
                                       std::copysign(1, rem_point.y()));
        weight = rem_point.norm();

      } else if (std::abs(rem_point.x()) > std::abs(rem_point.y())) {
        current_point =
            nn_point + Eigen::Vector2i(std::copysign(1, rem_point.x()), 0.0);
        opposite_point =
            nn_point - Eigen::Vector2i(std::copysign(1, rem_point.x()), 0.0);
        weight = rem_point.norm();
      } else {
        current_point =
            nn_point + Eigen::Vector2i(0.0, std::copysign(1, rem_point.y()));
        opposite_point =
            nn_point - Eigen::Vector2i(0.0, std::copysign(1, rem_point.y()));
        weight = rem_point.norm();
      }

      if ((nn_point.x() < 0) || (nn_point.y() < 0) ||
          (nn_point.x() >= map.cols()) || (nn_point.y() >= map.rows()) ||
          (current_point.x() < 0) || (current_point.y() < 0) ||
          (current_point.x() >= map.cols()) ||
          (current_point.y() >= map.rows()) || (opposite_point.x() < 0) ||
          (opposite_point.y() < 0) || (opposite_point.x() >= map.cols()) ||
          (opposite_point.y() >= map.rows())) {
        output->at<float>(v, u) = 0;
      } else {

        float a = input.at<float>(nn_point.y(), nn_point.x());
        float diff_curr = input.at<float>(current_point.y(), current_point.x()) - a;
        float diff_opp = input.at<float>(opposite_point.y(), opposite_point.x()) - a;

        if(std::abs(diff_curr) > std::abs(diff_opp)){
          diff_curr = std::copysign(diff_opp, diff_curr);
        }
        output->at<float>(v, u) =
            (1 - weight) * input.at<float>(nn_point.y(), nn_point.x()) +
            weight * input.at<float>(current_point.y(), current_point.x());
      }
    }
  }
}

void Tester::interpolateBilinear(const cv::Mat input,
                                 const Interpolator::DistortionMap map,
                                 cv::Mat* output) {
  output->create(map.cols(), map.rows(), CV_32FC1);
  for (size_t v = 0; v < map.rows(); ++v) {
    for (size_t u = 0; u < map.cols(); ++u) {
      output->at<float>(v, u) = 0;

      Eigen::Vector2d point = map(v, u);
      Eigen::Vector2i nn_point(std::floor(point.x()), std::floor(point.y()));
      Eigen::Vector2d rem_point = point - nn_point.cast<double>();

      for (size_t i = 0; i <= 1; ++i) {
        for (size_t j = 0; j <= 1; ++j) {
          Eigen::Vector2i current_point(nn_point.x() + i, nn_point.y() + j);

          if ((current_point.x() < 0) || (current_point.y() < 0) ||
              (current_point.x() >= map.cols()) ||
              (current_point.y() >= map.rows())) {
            output->at<float>(v, u) = 0;
          } else {
            output->at<float>(v, u) +=
                std::abs((1.0 - i - rem_point.x()) *
                         (1.0 - j - rem_point.y())) *
                input.at<float>(current_point.y(), current_point.x());
          }
        }
      }
    }
  }
}

// http://blog.demofox.org/2015/08/15/resizing-images-with-bicubic-interpolation
// t is a value that goes from 0 to 1 to interpolate in a C1 continuous way
// across uniformly sampled data points. when t is 0, this will return B.  When
// t is 1, this will return C.  Inbetween values will return an interpolation
// between B and C.  A and B are used to calculate slopes at the edges.
float Tester::CubicHermite(float A, float B, float C, float D, float t) {
  float a = -A / 2.0f + (3.0f * B) / 2.0f - (3.0f * C) / 2.0f + D / 2.0f;
  float b = A - (5.0f * B) / 2.0f + 2.0f * C - D / 2.0f;
  float c = -A / 2.0f + C / 2.0f;
  float d = B;

  return a * t * t * t + b * t * t + c * t + d;
}

void Tester::interpolateBicubic(const cv::Mat input,
                                const Interpolator::DistortionMap map,
                                cv::Mat* output) {
  output->create(map.cols(), map.rows(), CV_32FC1);
  for (size_t v = 0; v < map.rows(); ++v) {
    for (size_t u = 0; u < map.cols(); ++u) {
      output->at<float>(v, u) = 0;

      Eigen::Vector2d point = map(v, u);
      Eigen::Vector2i nn_point(std::floor(point.x()), std::floor(point.y()));
      Eigen::Vector2d rem_point = point - nn_point.cast<double>();

      std::vector<float> x_vals;
      for (int i = -1; i <= 2; ++i) {
        std::vector<float> y_vals;
        for (int j = -1; j <= 2; ++j) {
          Eigen::Vector2i current_point(nn_point.x() + i, nn_point.y() + j);

          if ((current_point.x() < 0) || (current_point.y() < 0) ||
              (current_point.x() >= map.cols()) ||
              (current_point.y() >= map.rows())) {
            y_vals.push_back(0.0);
          } else {
            y_vals.push_back(
                input.at<float>(current_point.y(), current_point.x()));
          }
        }
        x_vals.push_back(CubicHermite(y_vals[0], y_vals[1], y_vals[2],
                                      y_vals[3], rem_point.y()));
      }
      output->at<float>(v, u) = CubicHermite(x_vals[0], x_vals[1], x_vals[2],
                                             x_vals[3], rem_point.x());
    }
  }
}

void Tester::generateRotationMap(
    const cv::Size& image_size, const double rotation_angle,
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
          pixel_location.x() * std::cos(-rotation_angle) -
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
  images["nearest neighbor"] = float_image.clone();
  images["bilinear"] = float_image.clone();
  images["nearest linear"] = float_image.clone();
  images["bicubic"] = float_image.clone();
  images["lanczos4"] = float_image.clone();

  // get remapping
  Interpolator::DistortionMap map;
  generateRotationMap(input_image.size(), rotation_angle, &map);

  // rotate images
  for (size_t i = 0; i < number_of_rotations; ++i) {
    cv::Mat temp;

    interpolateNearestNeighbor(images["nearest neighbor"], map, &temp);
    images["nearest neighbor"] = temp.clone();

    interpolateBilinear(images["bilinear"], map, &temp);
    images["bilinear"] = temp.clone();

    interpolateNearestLinear(images["nearest linear"], map, &temp);
    images["nearest linear"] = temp.clone();

    interpolateBicubic(images["bicubic"], map, &temp);
    images["bicubic"] = temp.clone();
    /*
    cv::remap(images["lanczos4"], temp, map_x, map_y, cv::INTER_LANCZOS4);
    images["lanczos4"] = temp.clone();*/
  }

  // lossless compression
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);

  // save results
  cv::Mat temp;

  images["nearest neighbor"].convertTo(temp, CV_8UC1);
  cv::imwrite(output_image_directory + "/nearest_neighbor.png", temp,
              compression_params);

  images["bilinear"].convertTo(temp, CV_8UC1);
  cv::imwrite(output_image_directory + "/bilinear.png", temp,
              compression_params);

  images["nearest linear"].convertTo(temp, CV_8UC1);
  cv::imwrite(output_image_directory + "/nearest_linear.png", temp,
              compression_params);

  images["bicubic"].convertTo(temp, CV_8UC1);
  cv::imwrite(output_image_directory + "/bicubic.png", temp,
              compression_params);

  images["lanczos4"].convertTo(temp, CV_8UC1);
  cv::imwrite(output_image_directory + "/lanczos4.png", temp,
              compression_params);

  exit(0);
}
}