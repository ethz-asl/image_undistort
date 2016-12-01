#include <image_undistort/stereo_undistort.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace image_undistort {

class StereoUndistortNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    stereo_undistort_ = std::make_shared<StereoUndistort>(
        getNodeHandle(), getPrivateNodeHandle());
  }

  std::shared_ptr<StereoUndistort> stereo_undistort_;
};
}

PLUGINLIB_DECLARE_CLASS(image_undistort, StereoUndistortNodelet,
                        image_undistort::StereoUndistortNodelet,
                        nodelet::Nodelet);