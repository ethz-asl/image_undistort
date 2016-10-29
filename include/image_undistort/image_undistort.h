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

// Default values

// queue size
constexpr int kQueueSize = 100;
// true to load cam_info from kalibr yaml file, false to get it from a
// cam_info topic
constexpr bool kDefaultCamInfoFromYaml = true;
// namespace to use when reading yaml file
const std::string kDefaultCamNameSpace = "";
// true to process images, false if you only wish to generate a cam_info topic
// from a yaml file (the image topic must still be subscribed to so that the
// cam_info message is published at the correct times).
constexpr bool kDefaultProcessImage = true;
// true to undistort, false if you only wish to use the other operations
// (zoom,
// downsampling, center focus, etc)
constexpr bool kDefaultUndistortImage = true;
// if true the image is translated so that center of the image is the location
// of the focal point
constexpr bool kDefaultCenterFocus = false;
// zoom factor to apply to image (does not change size of image)
constexpr double kDefaultZoomFactor = 1.0;
// factor to increase image size by (output_image_size = ceil( resize_factor *
// input_image_size )
constexpr double kDefaultResizeFactor = 1.0;
// downsamples output rate if <= 1, every frame is processed.
constexpr int kDefaultProcessEveryNthFrame = 1;

class ImageUndistort {
 public:
  ImageUndistort(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

 private:
  void imageMsgToCvMat(const sensor_msgs::ImageConstPtr& image_msg,
                       cv::Mat* image);

  void updateCamInfo(const sensor_msgs::CameraInfo& new_raw_cam_info);

  bool loadCamParams(const std::string& cam_ns,
                     sensor_msgs::CameraInfo* new_raw_cam_info);

  void imageCallback(const sensor_msgs::ImageConstPtr& raw_image_msg);

  void cameraCallback(const sensor_msgs::ImageConstPtr& raw_image_msg,
                      const sensor_msgs::CameraInfoConstPtr& new_raw_cam_info);

  // nodes
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::ImageTransport it_;

  // subscribers
  image_transport::Subscriber raw_image_sub_;
  image_transport::CameraSubscriber raw_camera_sub_;

  // publishers
  image_transport::CameraPublisher undistorted_camera_pub_;
  ros::Publisher cam_info_pub_;

  // undistorter
  std::shared_ptr<Undistorter> undistorter_ptr_;

  // camera info
  sensor_msgs::CameraInfo raw_cam_info_;
  sensor_msgs::CameraInfo undistorted_cam_info_;

  // other variables
  int queue_size_;
  bool process_image_;
  bool undistort_image_;
  bool center_focus_;
  double zoom_factor_;
  double resize_factor_;
  int process_every_nth_frame_;

  int frame_counter_;
};

#endif