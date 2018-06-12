#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "image_undistort/image_undistort.h"

namespace image_undistort {

class ImageUndistortNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      image_undistort_ = std::make_shared<ImageUndistort>(
          getNodeHandle(), getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<ImageUndistort> image_undistort_;
};
}  // namespace image_undistort

PLUGINLIB_EXPORT_CLASS(image_undistort::ImageUndistortNodelet,
                       nodelet::Nodelet);
