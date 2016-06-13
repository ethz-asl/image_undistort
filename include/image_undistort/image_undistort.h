#ifndef IMAGE_UNDISTORT_H
#define IMAGE_UNDISTORT_H

#include <stdio.h>
#include <Eigen/Eigen>

// ros
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <image_undistort/undistorter.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/opencv.hpp>

class ImageUndistort {
 public:
  ImageUndistort(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

 private:

  void imageMsgToCvMat(const sensor_msgs::ImageConstPtr& image_msg,
                       cv::Mat* image);

  void newFrameCallback(const sensor_msgs::ImageConstPtr& image_msg);

  void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_msg);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // filters
  ros::Subscriber image_sub_;
  ros::Subscriber cam_info_sub_;

  // publishers
  ros::Publisher image_pub_;

  //undistorter
  std::shared_ptr<Undistorter> undistorter_ptr_;

};

// Default values
constexpr size_t kQueueSize = 100;
constexpr double kDefaultZoom = 1.0;

#endif