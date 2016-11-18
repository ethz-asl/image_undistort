#include <image_undistort/image_undistort.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

namespace image_undistort {

class ImageUndistortNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    image_undistort_ = std::make_shared<ImageUndistort>(getNodeHandle(),
                                                        getPrivateNodeHandle());
  }

  std::shared_ptr<ImageUndistort> image_undistort_;
};
}

PLUGINLIB_DECLARE_CLASS(image_undistort, ImageUndistortNodelet,
                        image_undistort::ImageUndistortNodelet, nodelet::Nodelet);