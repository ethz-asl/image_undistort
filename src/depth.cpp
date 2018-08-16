#include "image_undistort/depth.h"

namespace image_undistort {

// needed so that topic filters can be initialized in the constructor
int Depth::getQueueSize() const {
  int queue_size;
  nh_private_.param("queue_size", queue_size, kDepthQueueSize);
  if (queue_size < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size = 1;
  }
  return queue_size;
}

Depth::Depth(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      it_(nh_),
      queue_size_(getQueueSize()),
      first_image_sub_(it_, "rect/first/image", queue_size_),
      second_image_sub_(it_, "rect/second/image", queue_size_),
      first_camera_info_sub_(nh_, "rect/first/camera_info", queue_size_),
      second_camera_info_sub_(nh_, "rect/second/camera_info", queue_size_),
      camera_sync_(CameraSyncPolicy(queue_size_), first_image_sub_,
                   second_image_sub_, first_camera_info_sub_,
                   second_camera_info_sub_) {
  std::string pre_filter_type_string;
  nh_private_.param("pre_filter_type", pre_filter_type_string, kPreFilterType);

  if (pre_filter_type_string == std::string("xsobel")) {
    pre_filter_type_ = cv::StereoBM::PREFILTER_XSOBEL;
  } else if (pre_filter_type_string == std::string("normalized_response")) {
    pre_filter_type_ = cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE;
  } else {
    throw std::runtime_error(
        "Unrecognized prefilter type, choices are 'xsobel' or "
        "'normalized_response'");
  }

  // general stereo parameters
  nh_private_.param("min_disparity", min_disparity_, kMinDisparity);
  nh_private_.param("num_disparities", num_disparities_, kNumDisparities);
  nh_private_.param("pre_filter_cap", pre_filter_cap_, kPreFilterCap);
  nh_private_.param("uniqueness_ratio", uniqueness_ratio_, kUniquenessRatio);
  nh_private_.param("speckle_range", speckle_range_, kSpeckleRange);
  nh_private_.param("speckle_window_size", speckle_window_size_,
                    kSpeckleWindowSize);
  nh_private_.param("sad_window_size", sad_window_size_, kSADWindowSize);

  // bm parameters
  nh_private_.param("texture_threshold", texture_threshold_, kTextureThreshold);
  nh_private_.param("pre_filter_size", pre_filter_size_, kPreFilterSize);

  // sgbm parameters
  nh_private_.param("use_sgbm", use_sgbm_, kUseSGBM);
  nh_private_.param("p1", p1_, kP1);
  nh_private_.param("p2", p2_, kP2);
  nh_private_.param("disp_12_max_diff", disp_12_max_diff_, kDisp12MaxDiff);
  nh_private_.param("use_mode_HH", use_mode_HH_, kUseHHMode);

  nh_private_.param("do_median_blur", do_median_blur_, kDoMedianBlur);

  camera_sync_.registerCallback(
      boost::bind(&Depth::camerasCallback, this, _1, _2, _3, _4));

  disparity_pub_ = it_.advertise("disparity/image", queue_size_);
  pointcloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("pointcloud", queue_size_);
  freespace_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "freespace_pointcloud", queue_size_);
}

// simply replaces invalid disparity values with a valid value found by scanning
// horizontally (note: if disparity values are already valid or if no valid
// value can be found int_max is inserted)
void Depth::fillDisparityFromSide(const cv::Mat& input_disparity,
                                  const cv::Mat& valid, const bool& from_left,
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

void Depth::bulidFilledDisparityImage(const cv::Mat& input_disparity,
                                      cv::Mat* disparity_filled,
                                      cv::Mat* input_valid) const {
  // mark valid pixels
  *input_valid = cv::Mat(input_disparity.rows, input_disparity.cols, CV_8U);

  int side_bound = sad_window_size_ / 2;

  for (size_t y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
    for (size_t x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
      // the last check is because the sky has a bad habit of having a disparity
      // at just less than the max disparity
      if ((x_pixels < side_bound + min_disparity_ + num_disparities_) ||
          (y_pixels < side_bound) ||
          (x_pixels > (input_disparity.cols - side_bound)) ||
          (y_pixels > (input_disparity.rows - side_bound)) ||
          (input_disparity.at<int16_t>(y_pixels, x_pixels) < 0) ||
          (input_disparity.at<int16_t>(y_pixels, x_pixels) >=
           (min_disparity_ + num_disparities_ - 1) * 16)) {
        input_valid->at<uint8_t>(y_pixels, x_pixels) = 0;
      } else {
        input_valid->at<uint8_t>(y_pixels, x_pixels) = 1;
      }
    }
  }

  // erode by size of SAD window, this prevents issues with background pixels
  // being given the same depth as neighboring objects in the foreground.
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(sad_window_size_, sad_window_size_));
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

void Depth::calcPointCloud(
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

  int side_bound = sad_window_size_ / 2;

  // build pointcloud
  for (int y_pixels = side_bound; y_pixels < input_disparity.rows - side_bound;
       ++y_pixels) {
    for (int x_pixels = side_bound + min_disparity_ + num_disparities_;
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

void Depth::calcDisparityImage(
    const sensor_msgs::ImageConstPtr& left_image_msg,
    const sensor_msgs::ImageConstPtr& right_image_msg,
    cv_bridge::CvImagePtr disparity_ptr) const {
  cv_bridge::CvImageConstPtr left_ptr =
      cv_bridge::toCvShare(left_image_msg, "mono8");

  cv_bridge::CvImageConstPtr right_ptr =
      cv_bridge::toCvShare(right_image_msg, "mono8");

#if (defined(CV_VERSION_EPOCH) && CV_VERSION_EPOCH == 2)

  if (use_sgbm_) {
    std::shared_ptr<cv::StereoSGBM> left_matcher =
        std::make_shared<cv::StereoSGBM>();

    left_matcher->numberOfDisparities = num_disparities_;
    left_matcher->SADWindowSize = sad_window_size_;
    left_matcher->preFilterCap = pre_filter_cap_;
    left_matcher->minDisparity = min_disparity_;
    left_matcher->uniquenessRatio = uniqueness_ratio_;
    left_matcher->speckleRange = speckle_range_;
    left_matcher->speckleWindowSize = speckle_window_size_;
    left_matcher->P1 = p1_;
    left_matcher->P2 = p2_;
    left_matcher->disp12MaxDiff = disp_12_max_diff_;
    left_matcher->fullDP = use_mode_HH_;
    left_matcher->operator()(left_ptr->image, right_ptr->image,
                             disparity_ptr->image);
  } else {
    std::shared_ptr<cv::StereoBM> left_matcher =
        std::make_shared<cv::StereoBM>();

    left_matcher->state->numberOfDisparities = num_disparities_;
    left_matcher->state->SADWindowSize = sad_window_size_;
    left_matcher->state->preFilterCap = pre_filter_cap_;
    left_matcher->state->preFilterSize = pre_filter_size_;
    left_matcher->state->minDisparity = min_disparity_;
    left_matcher->state->textureThreshold = texture_threshold_;
    left_matcher->state->uniquenessRatio = uniqueness_ratio_;
    left_matcher->state->speckleRange = speckle_range_;
    left_matcher->state->speckleWindowSize = speckle_window_size_;
    left_matcher->operator()(left_ptr->image, right_ptr->image,
                             disparity_ptr->image);
  }
#else
  if (use_sgbm_) {
    int mode;
    if (use_mode_HH_) {
      mode = cv::StereoSGBM::MODE_HH;
    } else {
      mode = cv::StereoSGBM::MODE_SGBM;
    }

    cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(
        min_disparity_, num_disparities_, sad_window_size_, p1_, p2_,
        disp_12_max_diff_, pre_filter_cap_, uniqueness_ratio_,
        speckle_window_size_, speckle_range_, mode);

    left_matcher->compute(left_ptr->image, right_ptr->image,
                          disparity_ptr->image);

  } else {
    cv::Ptr<cv::StereoBM> left_matcher =
        cv::StereoBM::create(num_disparities_, sad_window_size_);

    left_matcher->setPreFilterType(pre_filter_type_);
    left_matcher->setPreFilterSize(pre_filter_size_);
    left_matcher->setPreFilterCap(pre_filter_cap_);
    left_matcher->setMinDisparity(min_disparity_);
    left_matcher->setTextureThreshold(texture_threshold_);
    left_matcher->setUniquenessRatio(uniqueness_ratio_);
    left_matcher->setSpeckleRange(speckle_range_);
    left_matcher->setSpeckleWindowSize(speckle_window_size_);
    left_matcher->compute(left_ptr->image, right_ptr->image,
                          disparity_ptr->image);
  }
#endif

  if(do_median_blur_){
    cv::medianBlur(disparity_ptr->image, disparity_ptr->image, 5);
  }
}

bool Depth::processCameraInfo(
    const sensor_msgs::CameraInfoConstPtr& first_camera_info,
    const sensor_msgs::CameraInfoConstPtr& second_camera_info, double* baseline,
    double* focal_length, bool* first_is_left, int* cx, int* cy) {
  if (first_camera_info->height != second_camera_info->height) {
    ROS_ERROR("Image heights do not match");
    return false;
  }
  if (first_camera_info->width != second_camera_info->width) {
    ROS_ERROR("Image widths do not match");
    return false;
  }

  for (double d : first_camera_info->D) {
    if (!ApproxEq(d, 0)) {
      ROS_ERROR("First image has non-zero distortion");
      return false;
    }
  }
  for (double d : second_camera_info->D) {
    if (!ApproxEq(d, 0)) {
      ROS_ERROR("Second image has non-zero distortion");
      return false;
    }
  }

  for (size_t i = 0; i < 12; ++i) {
    if ((i != 3) &&
        !ApproxEq(first_camera_info->P[i], second_camera_info->P[i])) {
      ROS_ERROR("Image P matrices must match (excluding x offset)");
      return false;
    }
  }

  if (!ApproxEq(first_camera_info->P[1], 0) ||
      !ApproxEq(first_camera_info->P[4], 0)) {
    ROS_ERROR("Image P matrix contains skew");
    return false;
  }

  if (!ApproxEq(first_camera_info->P[0], first_camera_info->P[5])) {
    ROS_ERROR("Image P matrix has different values for Fx and Fy");
    return false;
  }

  if (first_camera_info->P[0] <= 0) {
    ROS_ERROR("Focal length must be greater than 0");
    return false;
  }

  // downgraded to warning so that the color images of the KITTI dataset can be
  // processed
  if (!ApproxEq(first_camera_info->P[8], 0) ||
      !ApproxEq(first_camera_info->P[9], 0) ||
      !ApproxEq(first_camera_info->P[10], 1) ||
      !ApproxEq(first_camera_info->P[11], 0)) {
    ROS_WARN_ONCE(
        "Image P matrix does not end in [0,0,1,0], these values will be "
        "ignored");
  }

  // again downgraded to warning because KITTI has ugly matrices
  if (!ApproxEq(first_camera_info->P[7], 0)) {
    ROS_WARN_ONCE("P contains Y offset, this value will be ignored");
  }

  *focal_length = first_camera_info->P[0];
  *baseline = (second_camera_info->P[3] - first_camera_info->P[3]) /
              first_camera_info->P[0];
  if (*baseline > 0) {
    *first_is_left = false;
  } else {
    *first_is_left = true;
    *baseline *= -1;
  }
  *cx = first_camera_info->P[2];
  *cy = first_camera_info->P[6];

  return true;
}

bool Depth::ApproxEq(double A, double B) { return (std::abs(A - B) <= kDelta); }

void Depth::camerasCallback(
    const sensor_msgs::ImageConstPtr& first_image_msg,
    const sensor_msgs::ImageConstPtr& second_image_msg,
    const sensor_msgs::CameraInfoConstPtr& first_camera_info,
    const sensor_msgs::CameraInfoConstPtr& second_camera_info) {
  double baseline, focal_length;
  bool first_is_left;
  int cx, cy;

  if (!processCameraInfo(first_camera_info, second_camera_info, &baseline,
                         &focal_length, &first_is_left, &cx, &cy)) {
    ROS_ERROR("Camera info processing failed, skipping disparity generation");
    return;
  }

  sensor_msgs::ImageConstPtr left_image_msg;
  sensor_msgs::ImageConstPtr right_image_msg;

  if (first_is_left) {
    left_image_msg = first_image_msg;
    right_image_msg = second_image_msg;
  } else {
    left_image_msg = second_image_msg;
    right_image_msg = first_image_msg;
  }

  cv::Mat disparity_image =
      cv::Mat(left_image_msg->height, left_image_msg->width, CV_16S);
  cv_bridge::CvImagePtr disparity_ptr(new cv_bridge::CvImage(
      left_image_msg->header, "mono16", disparity_image));

  calcDisparityImage(left_image_msg, right_image_msg, disparity_ptr);
  disparity_pub_.publish(*(disparity_ptr->toImageMsg()));

  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB> freespace_pointcloud;
  cv_bridge::CvImageConstPtr left_ptr = cv_bridge::toCvShare(left_image_msg);
  calcPointCloud(disparity_ptr->image, left_ptr->image, baseline, focal_length,
                 cx, cy, &pointcloud, &freespace_pointcloud);

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  pointcloud_msg.header = left_image_msg->header;
  pointcloud_pub_.publish(pointcloud_msg);

  sensor_msgs::PointCloud2 freespace_pointcloud_msg;
  pcl::toROSMsg(freespace_pointcloud, freespace_pointcloud_msg);
  freespace_pointcloud_msg.header = left_image_msg->header;
  freespace_pointcloud_pub_.publish(freespace_pointcloud_msg);
}
}
