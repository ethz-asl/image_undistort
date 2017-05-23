#ifndef IMAGE_UNDISTORT_DISPARITY_H
#define IMAGE_UNDISTORT_DISPARITY_H

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>


#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>

#include "image_undistort/disparity_kernel.h"

namespace image_undistort {

// Default values

// queue size
constexpr int kQueueSize = 100;

class Disparity {
 public:
  Disparity(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void imagesCallback(const sensor_msgs::ImageConstPtr& left_image_msg_in,
                      const sensor_msgs::ImageConstPtr& right_image_msg_in);

 private:
  int getQueueSize() const;

  // nodes
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  image_transport::ImageTransport it_;

  // subscribers
  image_transport::SubscriberFilter left_image_sub_;
  image_transport::SubscriberFilter right_image_sub_;

  // publishers
  image_transport::Publisher disparity_pub_;

  // filters
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      ImageSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy>>
      image_sync_ptr_;

  int queue_size_;
};
}

#endif
