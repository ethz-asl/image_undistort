#ifndef IMAGE_UNDISTORT_POINT_TO_BEARING_NODELET_H
#define IMAGE_UNDISTORT_POINT_TO_BEARING_NODELET_H

#include <stdio.h>
#include <Eigen/Dense>

#include <nodelet/nodelet.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/CameraInfo.h>

#include <nlopt.hpp>

#include "image_undistort/undistorter.h"

namespace image_undistort {

// Default values

// queue size
constexpr int kQueueSize = 10;
// true to load cam_info from ros parameters, false to get it from a
// cam_info topic
constexpr bool kDefaultCameraInfoFromROSParams = false;
// namespace to use when loading camera parameters from ros params
const std::string kDefaultCameraNamespace = "camera";

class PointToBearing {
 public:
  PointToBearing(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  static void optimizeForBearingVector(
      const InputCameraParameters& camera_parameters,
      const Eigen::Vector2d& pixel_location, Eigen::Vector3d* bearing);

  static double bearingProjectionError(const std::vector<double>& values,
                                       std::vector<double>& grad, void* data);

 private:
  void imagePointCallback(
      const geometry_msgs::PointStampedConstPtr& image_point);

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);

  // nodes
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // subscribers
  ros::Subscriber camera_info_sub_;
  ros::Subscriber image_point_sub_;

  // publishers
  ros::Publisher bearing_pub_;

  // camera info
  std::shared_ptr<InputCameraParameters> camera_parameters_ptr_;

  // other variables
  int queue_size_;
};
}

#endif
