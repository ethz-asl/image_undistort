#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "image_undistort/disparity.h"

namespace image_undistort {

class DisparityNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    disparity_ = std::make_shared<Disparity>(
        getNodeHandle(), getPrivateNodeHandle());
  }

  std::shared_ptr<Disparity> disparity_;
};
}

PLUGINLIB_DECLARE_CLASS(image_undistort, DisparityNodelet,
                        image_undistort::DisparityNodelet,
                        nodelet::Nodelet);