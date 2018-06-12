#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "image_undistort/stereo_undistort.h"

namespace image_undistort {

class StereoUndistortNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      stereo_undistort_ = std::make_shared<StereoUndistort>(
          getNodeHandle(), getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<StereoUndistort> stereo_undistort_;
};
}  // namespace image_undistort

PLUGINLIB_EXPORT_CLASS(image_undistort::StereoUndistortNodelet,
                       nodelet::Nodelet);
