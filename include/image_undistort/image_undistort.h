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
// true to load input cam_info from a yaml file, false to get it from a cam_info
// topic
constexpr bool kDefaultInputCameraInfoFromYaml = true;
// true to load output cam_info from a yaml file, false to copy the input
// cam_info (with distortion removed, unless undistort_image is false)
constexpr bool kDefaultOutputCameraInfoFromYaml = false;
// namespace to use when reading input yaml file
const std::string kDefaultInputCameraNameSpace = "";
// namespace to use when reading output yaml file (cannot be the same as input
// namespace)
const std::string kDefaultOutputCameraNameSpace = "output_camera";
// true to process images, false if you only wish to generate a cam_info topic
// from a yaml file (the image topic must still be subscribed to so that the
// cam_info message is published at the correct times).
constexpr bool kDefaultProcessImage = true;
// true to undistort, false if you only wish to modify the intrinsics
constexpr bool kDefaultUndistortImage = true;
// downsamples output rate if <= 1, every frame is processed.
constexpr int kDefaultProcessEveryNthFrame = 1;

class ImageUndistort {
 public:
  ImageUndistort(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

 private:
  void imageMsgToCvMat(const sensor_msgs::ImageConstPtr& image_msg,
                       cv::Mat* image);

  void updateCameraInfo(const sensor_msgs::CameraInfo& camera_info);

  bool loadCameraParameters(const bool is_input,
                            sensor_msgs::CameraInfo* loaded_camera_info,
                            std::string* image_topic);

  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg_in);

  void cameraCallback(const sensor_msgs::ImageConstPtr& image_msg_in,
                      const sensor_msgs::CameraInfoConstPtr& cam_info_in);

  template <typename Derived>
  static bool xmlRpcToMatrix(const XmlRpc::XmlRpcValue& const_input,
                             Eigen::MatrixBase<Derived>* output) {
    // A local copy is required as the methods that allow you to access the
    // XmlRpc values as doubles are not const and so cannot be used with the
    // const ref
    XmlRpc::XmlRpcValue input = const_input;

    if (input.size() != output->rows()) {
      ROS_ERROR_STREAM("Loaded matrix has "
                       << input.size() << " rows, expected " << output->rows());
      return false;
    }
    for (size_t i = 0; i < output->rows(); ++i) {
      if (input[i].size() != output->cols()) {
        ROS_ERROR_STREAM("Loaded matrix has "
                         << input[i].size() << " columns in its " << i
                         << " row, expected " << output->cols());
        return false;
      }
      for (size_t j = 0; j < output->cols(); ++j) {
        output->coeffRef(i, j) = input[i][j];
      }
    }
    return true;
  }

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
  bool output_camera_info_from_yaml_;
  int process_every_nth_frame_;

  int frame_counter_;
};

#endif