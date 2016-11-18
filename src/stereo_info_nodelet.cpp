#include <image_undistort/stereo_info.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace image_undistort {

class StereoInfoNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    stereo_info_ = std::make_shared<StereoInfo>(getNodeHandle(),
                                                        getPrivateNodeHandle());
  }

  std::shared_ptr<StereoInfo> stereo_info_;
};
}

PLUGINLIB_DECLARE_CLASS(image_undistort, StereoInfoNodelet,
                        image_undistort::StereoInfoNodelet, nodelet::Nodelet);