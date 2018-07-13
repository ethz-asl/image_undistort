#ifndef IMAGE_UNDISTORT_DISPARITY_H
#define IMAGE_UNDISTORT_DISPARITY_H

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include "image_undistort/depth_generator.h"

namespace image_undistort {

// Default values

// queue size
constexpr int kQueueSize = 10;

class Depth {
 public:
  Depth(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void camerasCallback(
      const sensor_msgs::ImageConstPtr& first_image_msg_in,
      const sensor_msgs::ImageConstPtr& second_image_msg_in,
      const sensor_msgs::CameraInfoConstPtr& first_camera_info_msg_in,
      const sensor_msgs::CameraInfoConstPtr& second_camera_info_msg_in);

 private:
  int getQueueSize() const;

  // nodes
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  image_transport::ImageTransport it_;

  int queue_size_;

  // subscribers
  image_transport::SubscriberFilter first_image_sub_;
  image_transport::SubscriberFilter second_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> first_camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> second_camera_info_sub_;

  // publishers
  image_transport::Publisher disparity_pub_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher freespace_pointcloud_pub_;

  // filters
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
      sensor_msgs::CameraInfo>
      CameraSyncPolicy;
  message_filters::Synchronizer<CameraSyncPolicy> camera_sync_;

  std::shared_ptr<DepthGenerator> depth_gen_;
};
}

#endif
