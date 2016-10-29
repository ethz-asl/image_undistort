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
constexpr bool kDefaultCameraInfoFromYaml = true;
// namespace to use when reading yaml file
const std::string kDefaultCameraNameSpace = "";
// true to process images, false if you only wish to generate a cam_info topic
// from a yaml file (the image topic must still be subscribed to so that the
// cam_info message is published at the correct times).
constexpr bool kDefaultProcessImage = true;
// true to undistort, false if you only wish to modify the intrinsics
constexpr bool kDefaultUndistortImage = true;
// true if the output image size should not match the input images (If true, a
// parameter output_image_size which holds two ints (width, height) must also be
// given)
constexpr bool kDefaultModifyImageSize = false;
// true if the output image intrinsics should not match the input images (If
// true, a parameter output_intrinsics which holds four doubles (fx, fy, cx, cy)
// must also be given)
constexpr bool kDefaultModifyIntrinsics = false;
// downsamples output rate if <= 1, every frame is processed.
constexpr int kDefaultProcessEveryNthFrame = 1;

class ImageUndistort {
 public:
  ImageUndistort(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

 private:
  void imageMsgToCvMat(const sensor_msgs::ImageConstPtr& image_msg,
                       cv::Mat* image);

  void updateCameraInfo(const sensor_msgs::CameraInfo& camera_info_in);

  bool loadCameraParams(sensor_msgs::CameraInfo* loaded_camera_info);

  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg_in);

  void cameraCallback(const sensor_msgs::ImageConstPtr& image_msg_in,
                      const sensor_msgs::CameraInfoConstPtr& cam_info_in);

  // nodes
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::ImageTransport it_;

  // subscribers
  image_transport::Subscriber image_sub_;
  image_transport::CameraSubscriber camera_sub_;

  // publishers
  image_transport::CameraPublisher camera_pub_;
  ros::Publisher camera_info_pub_;

  // undistorter
  std::shared_ptr<Undistorter> undistorter_ptr_;

  // camera info
  sensor_msgs::CameraInfo camera_info_in_;
  sensor_msgs::CameraInfo camera_info_out_;

  // other variables
  int queue_size_;
  bool process_image_;
  bool undistort_image_;
  bool modify_image_size_;
  std::vector<int> output_image_size_;
  bool modify_intrinsics_;
  std::vector<double> output_intrinsics_;
  int process_every_nth_frame_;

  int frame_counter_;
};

#endif