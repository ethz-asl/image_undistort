#include "image_undistort/disparity.h"

namespace image_undistort {

// needed so that topic filters can be initalized in the constructor
int Disparity::getQueueSize() const {
  int queue_size;
  nh_private_.param("queue_size", queue_size, kQueueSize);
  if (queue_size < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size = 1;
  }
  return queue_size;
}

Disparity::Disparity(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      it_(nh_),
      queue_size_(getQueueSize()),
      left_image_sub_(it_, "undistorted/left/image", queue_size_),
      right_image_sub_(it_, "undistorted/right/image", queue_size_){

  disparity_pub_ = it_.advertise("disparity/image", queue_size_);
}


void Disparity::imagesCallback(
    const sensor_msgs::ImageConstPtr& left_image_msg_in,
    const sensor_msgs::ImageConstPtr& right_image_msg_in) {

  cv_bridge::CvImageConstPtr left_image_ptr =
  cv_bridge::toCvShare(left_image_msg_in, CV_8UC1);

  cv_bridge::CvImageConstPtr right_image_ptr =
  cv_bridge::toCvShare(right_image_msg_in, CV_8UC1);

  cv_bridge::CvImagePtr disparity_ptr(
      new cv_bridge::CvImage(image_in_ptr->header, CV_32S));

  disparity_ptr->header = left_image_ptr;


  disparity_pub_.publish(*(disparity_ptr->toImageMsg()));
}
}
