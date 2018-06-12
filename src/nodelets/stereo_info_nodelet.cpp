#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "image_undistort/stereo_info.h"

namespace image_undistort {

class StereoInfoNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      stereo_info_ =
          std::make_shared<StereoInfo>(getNodeHandle(), getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<StereoInfo> stereo_info_;
};
}  // namespace image_undistort

PLUGINLIB_EXPORT_CLASS(image_undistort::StereoInfoNodelet, nodelet::Nodelet);
