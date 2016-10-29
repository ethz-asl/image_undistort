#ifndef IMAGE_UNDISTORT_H
#define IMAGE_UNDISTORT_H

#include <stdio.h>
#include <Eigen/Eigen>

#include <image_transport/image_transport.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <image_undistort/undistorter.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

class ImageUndistort {
 public:
  ImageUndistort(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

 private:
  void imageMsgToCvMat(const sensor_msgs::ImageConstPtr& image_msg,
                       cv::Mat* image);

  void updateCamInfo(const sensor_msgs::CameraInfo& new_raw_cam_info);

  void undistortImage(const sensor_msgs::ImageConstPtr& raw_image_msg,
                      sensor_msgs::ImagePtr undistorted_image_msg_ptr);

  bool loadCamParams(const std::string& cam_ns,
                     sensor_msgs::CameraInfo* new_raw_cam_info);

  void imageCallback(const sensor_msgs::ImageConstPtr& raw_image_msg);

  void cameraCallback(const sensor_msgs::ImageConstPtr& raw_image_msg,
                      const sensor_msgs::CameraInfoConstPtr& new_raw_cam_info);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::ImageTransport it_;

  image_transport::Subscriber raw_image_sub_;
  image_transport::CameraSubscriber raw_camera_sub_;

  // publishers
  image_transport::CameraPublisher undistorted_camera_pub_;

  // undistorter
  std::shared_ptr<Undistorter> undistorter_ptr_;

  // camera info
  sensor_msgs::CameraInfo raw_cam_info_;
  sensor_msgs::CameraInfo undistorted_cam_info_;

  double zoom_;
};

// Default values

// queue size
constexpr size_t kQueueSize = 100;
// true to load cam_info from kalibr yaml file, false to get it from a cam_info
// topic
constexpr bool kDefaultCamInfoFromYaml = true;
// zoom to apply to the images
constexpr double kDefaultZoom = 1.0;
// namespace to look for camera topics in
const std::string kDefaultCamNameSpace = "";

#endif