#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "image_undistort/point_to_bearing.h"

namespace image_undistort {

class PointToBearingNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      point_to_bearing_ = std::make_shared<PointToBearing>(
          getNodeHandle(), getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<PointToBearing> point_to_bearing_;
};
}  // namespace image_undistort

PLUGINLIB_EXPORT_CLASS(image_undistort::PointToBearingNodelet,
                       nodelet::Nodelet);
