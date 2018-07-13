#include "image_undistort/depth.h"

namespace image_undistort {

// simply replaces invalid disparity values with a valid value found by scanning
// horizontally (note: if disparity values are already valid or if no valid
// value can be found int_max is inserted)
void DepthGenerator::fillDisparityFromSide(const cv::Mat& input_disparity,
                                           const cv::Mat& valid,
                                           const bool& from_left,
                                           cv::Mat* filled_disparity) {
  *filled_disparity =
      cv::Mat(input_disparity.rows, input_disparity.cols, CV_16S);

  for (size_t y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
    bool prev_valid = false;
    int16_t prev_value;

    for (size_t x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
      size_t x_scan;
      if (from_left) {
        x_scan = x_pixels;
      } else {
        x_scan = (input_disparity.cols - x_pixels - 1);
      }

      if (valid.at<uint8_t>(y_pixels, x_scan)) {
        prev_valid = true;
        prev_value = input_disparity.at<int16_t>(y_pixels, x_scan);
        filled_disparity->at<int16_t>(y_pixels, x_scan) =
            std::numeric_limits<int16_t>::max();
      } else if (prev_valid) {
        filled_disparity->at<int16_t>(y_pixels, x_scan) = prev_value;
      } else {
        filled_disparity->at<int16_t>(y_pixels, x_scan) =
            std::numeric_limits<int16_t>::max();
      }
    }
  }
}

void DepthGenerator::bulidFilledDisparityImage(const cv::Mat& input_disparity,
                                               cv::Mat* disparity_filled,
                                               cv::Mat* input_valid) const {
  // mark valid pixels
  *input_valid = cv::Mat(input_disparity.rows, input_disparity.cols, CV_8U);

  int side_bound = config_.sad_window_size / 2;

  for (size_t y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
    for (size_t x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
      // the last check is because the sky has a bad habit of having a disparity
      // at just less than the max disparity
      if ((x_pixels <
           side_bound + config_.min_disparity + config_.num_disparities) ||
          (y_pixels < side_bound) ||
          (x_pixels > (input_disparity.cols - side_bound)) ||
          (y_pixels > (input_disparity.rows - side_bound)) ||
          (input_disparity.at<int16_t>(y_pixels, x_pixels) < 0) ||
          (input_disparity.at<int16_t>(y_pixels, x_pixels) >=
           (config_.min_disparity + config_.num_disparities - 1) * 16)) {
        input_valid->at<uint8_t>(y_pixels, x_pixels) = 0;
      } else {
        input_valid->at<uint8_t>(y_pixels, x_pixels) = 1;
      }
    }
  }

  // erode by size of SAD window, this prevents issues with background pixels
  // being given the same depth as neighboring objects in the foreground.
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(config_.sad_window_size, config_.sad_window_size));
  cv::erode(*input_valid, *input_valid, kernel);

  // take a guess for the depth of the invalid pixels by scanning along the row
  // and giving them the same value as the closest horizontal point.
  cv::Mat disparity_filled_left, disparity_filled_right;
  fillDisparityFromSide(input_disparity, *input_valid, true,
                        &disparity_filled_left);
  fillDisparityFromSide(input_disparity, *input_valid, false,
                        &disparity_filled_right);

  // take the most conservative disparity of the two
  *disparity_filled = cv::max(disparity_filled_left, disparity_filled_right);

  // 0 disparity is valid but cannot have a depth associated with it, because of
  // this we take these points and replace them with a disparity of 1.
  for (size_t y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
    for (size_t x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
      if (input_disparity.at<int16_t>(y_pixels, x_pixels) == 0) {
        disparity_filled->at<int16_t>(y_pixels, x_pixels) = 1;
      }
    }
  }
}

void DepthGenerator::calcPointCloud(
    const cv::Mat& input_disparity, const cv::Mat& left_image,
    const double baseline, const double focal_length, const int cx,
    const int cy, pcl::PointCloud<pcl::PointXYZRGB>* pointcloud,
    pcl::PointCloud<pcl::PointXYZRGB>* freespace_pointcloud) {
  pointcloud->clear();
  freespace_pointcloud->clear();

  if (left_image.depth() != CV_8U) {
    ROS_ERROR(
        "Pointcloud generation is currently only supported on 8 bit images");
    return;
  }

  cv::Mat disparity_filled, input_valid;
  bulidFilledDisparityImage(input_disparity, &disparity_filled, &input_valid);

  int side_bound = config_.sad_window_size / 2;

  // build pointcloud
  for (int y_pixels = side_bound; y_pixels < input_disparity.rows - side_bound;
       ++y_pixels) {
    for (int x_pixels = side_bound + config_.min_disparity + config_.num_disparities;
         x_pixels < input_disparity.cols - side_bound; ++x_pixels) {
      const uint8_t& is_valid = input_valid.at<uint8_t>(y_pixels, x_pixels);
      const int16_t& input_value =
          input_disparity.at<int16_t>(y_pixels, x_pixels);
      const int16_t& filled_value =
          disparity_filled.at<int16_t>(y_pixels, x_pixels);

      bool freespace;
      double disparity_value;

      // if the filled disparity is valid it must be a freespace ray
      if (filled_value < std::numeric_limits<int16_t>::max()) {
        disparity_value = static_cast<double>(filled_value);
        freespace = true;
      }
      // else it is a normal ray
      else if (is_valid) {
        disparity_value = static_cast<double>(input_value);
        freespace = false;
      } else {
        continue;
      }

      pcl::PointXYZRGB point;

      // the 16* is needed as opencv stores disparity maps as 16 * the true
      // values
      point.z = (16 * focal_length * baseline) / disparity_value;
      point.x = point.z * (x_pixels - cx) / focal_length;
      point.y = point.z * (y_pixels - cy) / focal_length;

      if (left_image.channels() == 3) {
        const cv::Vec3b& color = left_image.at<cv::Vec3b>(y_pixels, x_pixels);
        point.b = color[0];
        point.g = color[1];
        point.r = color[2];
      } else if (left_image.channels() == 4) {
        const cv::Vec4b& color = left_image.at<cv::Vec4b>(y_pixels, x_pixels);
        point.b = color[0];
        point.g = color[1];
        point.r = color[2];
      } else {
        point.b = left_image.at<uint8_t>(y_pixels, x_pixels);
        point.g = point.b;
        point.r = point.b;
      }

      if (freespace) {
        freespace_pointcloud->push_back(point);
      } else {
        pointcloud->push_back(point);
      }
    }
  }
}

void DepthGenerator::calcDisparityImage(const cv::Mat& left_image,
                                        const cv::Mat& right_image,
                                        cv::Mat* disparity_ptr) const {
#if (defined(CV_VERSION_EPOCH) && CV_VERSION_EPOCH == 2)

  if (use_sgbm_) {
    std::shared_ptr<cv::StereoSGBM> left_matcher =
        std::make_shared<cv::StereoSGBM>();

    left_matcher->numberOfDisparities = config_.num_disparities;
    left_matcher->SADWindowSize = config_.sad_window_size;
    left_matcher->preFilterCap = config_.pre_filter_cap;
    left_matcher->minDisparity = config_.min_disparity;
    left_matcher->uniquenessRatio = config_.uniqueness_ratio;
    left_matcher->speckleRange = config_.speckle_range;
    left_matcher->speckleWindowSize = config_.speckle_window_size;
    left_matcher->P1 = config_.p1;
    left_matcher->P2 = config_.p2;
    left_matcher->disp12MaxDiff = config_.disp_12_max_diff;
    left_matcher->fullDP = config_.use_mode_HH;
    left_matcher->operator()(left_image, right_image, *disparity_ptr);
  } else {
    std::shared_ptr<cv::StereoBM> left_matcher =
        std::make_shared<cv::StereoBM>();

    left_matcher->state->numberOfDisparities = config_.num_disparities;
    left_matcher->state->SADWindowSize = config_.sad_window_size;
    left_matcher->state->preFilterCap = config_.pre_filter_cap;
    left_matcher->state->preFilterSize = config_.pre_filter_size;
    left_matcher->state->minDisparity = config_.min_disparity;
    left_matcher->state->textureThreshold = config_.texture_threshold;
    left_matcher->state->uniquenessRatio = config_.uniqueness_ratio;
    left_matcher->state->speckleRange = config_.speckle_range;
    left_matcher->state->speckleWindowSize = config_.speckle_window_size;
    left_matcher->operator()(left_image, right_image, *disparity_ptr);
  }
#else
  if (config_.use_sgbm) {
    int mode;
    if (config_.use_mode_HH) {
      mode = cv::StereoSGBM::MODE_HH;
    } else {
      mode = cv::StereoSGBM::MODE_SGBM;
    }

    cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(
        config_.min_disparity, config_.num_disparities, config_.sad_window_size, config_.p1, config_.p2,
        config_.disp_12_max_diff, config_.pre_filter_cap, config_.uniqueness_ratio,
        config_.speckle_window_size, config_.speckle_range, mode);

    left_matcher->compute(left_image, right_image, *disparity_ptr);

  } else {
    cv::Ptr<cv::StereoBM> left_matcher =
        cv::StereoBM::create(config_.num_disparities, config_.sad_window_size);

    left_matcher->setPreFilterType(config_.pre_filter_type);
    left_matcher->setPreFilterSize(config_.pre_filter_size);
    left_matcher->setPreFilterCap(config_.pre_filter_cap);
    left_matcher->setMinDisparity(config_.min_disparity);
    left_matcher->setTextureThreshold(config_.texture_threshold);
    left_matcher->setUniquenessRatio(config_.uniqueness_ratio);
    left_matcher->setSpeckleRange(config_.speckle_range);
    left_matcher->setSpeckleWindowSize(config_.speckle_window_size);
    left_matcher->compute(left_image, right_image, *disparity_ptr);
  }
#endif

  if (config_.do_median_blur) {
    cv::medianBlur(*disparity_ptr, *disparity_ptr, 5);
  }
}

bool DepthGenerator::processCameraInfo(
    const sensor_msgs::CameraInfo& first_camera_info,
    const sensor_msgs::CameraInfo& second_camera_info, double* baseline,
    double* focal_length, bool* first_is_left, int* cx, int* cy) {
  if (first_camera_info.height != second_camera_info.height) {
    ROS_ERROR("Image heights do not match");
    return false;
  }
  if (first_camera_info.width != second_camera_info.width) {
    ROS_ERROR("Image widths do not match");
    return false;
  }

  for (double d : first_camera_info.D) {
    if (!ApproxEq(d, 0)) {
      ROS_ERROR("First image has non-zero distortion");
      return false;
    }
  }
  for (double d : second_camera_info.D) {
    if (!ApproxEq(d, 0)) {
      ROS_ERROR("Second image has non-zero distortion");
      return false;
    }
  }

  for (size_t i = 0; i < 12; ++i) {
    if ((i != 3) &&
        !ApproxEq(first_camera_info.P[i], second_camera_info.P[i])) {
      ROS_ERROR("Image P matrices must match (excluding x offset)");
      return false;
    }
  }

  if (!ApproxEq(first_camera_info.P[1], 0) ||
      !ApproxEq(first_camera_info.P[4], 0)) {
    ROS_ERROR("Image P matrix contains skew");
    return false;
  }

  if (!ApproxEq(first_camera_info.P[0], first_camera_info.P[5])) {
    ROS_ERROR("Image P matrix has different values for Fx and Fy");
    return false;
  }

  if (first_camera_info.P[0] <= 0) {
    ROS_ERROR("Focal length must be greater than 0");
    return false;
  }

  // downgraded to warning so that the color images of the KITTI dataset can be
  // processed
  if (!ApproxEq(first_camera_info.P[8], 0) ||
      !ApproxEq(first_camera_info.P[9], 0) ||
      !ApproxEq(first_camera_info.P[10], 1) ||
      !ApproxEq(first_camera_info.P[11], 0)) {
    ROS_WARN_ONCE(
        "Image P matrix does not end in [0,0,1,0], these values will be "
        "ignored");
  }

  // again downgraded to warning because KITTI has ugly matrices
  if (!ApproxEq(first_camera_info.P[7], 0)) {
    ROS_WARN_ONCE("P contains Y offset, this value will be ignored");
  }

  *focal_length = first_camera_info.P[0];
  *baseline = (second_camera_info.P[3] - first_camera_info.P[3]) /
              first_camera_info.P[0];
  if (*baseline > 0) {
    *first_is_left = false;
  } else {
    *first_is_left = true;
    *baseline *= -1;
  }
  *cx = first_camera_info.P[2];
  *cy = first_camera_info.P[6];

  return true;
}

bool DepthGenerator::ApproxEq(double A, double B) {
  return (std::abs(A - B) <= kDelta);
}

}  // namespace image_undistort
