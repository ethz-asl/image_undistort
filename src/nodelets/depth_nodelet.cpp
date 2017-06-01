#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "image_undistort/depth.h"

namespace image_undistort {

class DepthNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    depth_ = std::make_shared<Depth>(getNodeHandle(), getPrivateNodeHandle());
  }

  std::shared_ptr<Depth> depth_;
};
}

PLUGINLIB_DECLARE_CLASS(image_undistort, DepthNodelet,
                        image_undistort::DepthNodelet, nodelet::Nodelet);