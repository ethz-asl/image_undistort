#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "image_undistort/depth.h"

namespace image_undistort {

class DepthNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      depth_ = std::make_shared<Depth>(getNodeHandle(), getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<Depth> depth_;
};
}  // namespace image_undistort

PLUGINLIB_EXPORT_CLASS(image_undistort::DepthNodelet, nodelet::Nodelet);
