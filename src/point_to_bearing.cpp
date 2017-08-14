#include "image_undistort/point_to_bearing.h"
#include "image_undistort/camera_parameters.h"
#include "image_undistort/undistorter.h"

namespace image_undistort {

PointToBearing::PointToBearing(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  // set parameters from ros
  bool camera_info_from_ros_params;
  nh_private_.param("camera_info_from_ros_params", camera_info_from_ros_params,
                    kDefaultCameraInfoFromROSParams);

  nh_private_.param("queue_size", queue_size_, kQueueSize);
  if (queue_size_ < 1) {
    ROS_ERROR("Queue size must be >= 1, setting to 1");
    queue_size_ = 1;
  }

  // setup subscribers
  std::string camera_namespace;
  if (camera_info_from_ros_params) {
    nh_private_.param("camera_namespace", camera_namespace,
                      kDefaultCameraNamespace);

    try {
      camera_parameters_ptr_ = std::make_shared<InputCameraParameters>(
          nh_private_, camera_namespace);
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
      ROS_FATAL("Loading of input camera parameters failed, exiting");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }

  } else {
    camera_info_sub_ = nh_.subscribe("camera_info", queue_size_,
                                     &PointToBearing::cameraInfoCallback, this);
  }

  image_point_sub_ = nh_.subscribe("image_point", queue_size_,
                                   &PointToBearing::imagePointCallback, this);

  // setup publishers
  bearing_pub_ =
      nh_.advertise<geometry_msgs::PointStamped>("bearing", queue_size_);
}

void PointToBearing::imagePointCallback(
    const geometry_msgs::PointStampedConstPtr& image_point) {
  if (camera_parameters_ptr_ == nullptr) {
    ROS_WARN("No camera calibration, cannot calculate bearing");
    return;
  }

  const Eigen::Vector2d pixel_location(image_point->point.x,
                                       image_point->point.y);

  Eigen::Vector3d bearing;
  optimizeForBearingVector(*camera_parameters_ptr_, pixel_location, &bearing);

  geometry_msgs::PointStamped bearing_msg;
  bearing_msg.header = image_point->header;
  bearing_msg.point.x = bearing.x();
  bearing_msg.point.y = bearing.y();
  bearing_msg.point.z = bearing.z();
  bearing_pub_.publish(bearing_msg);
}

void PointToBearing::optimizeForBearingVector(
    const InputCameraParameters& camera_parameters,
    const Eigen::Vector2d& pixel_location, Eigen::Vector3d* bearing) {
  const Eigen::Vector2d inital_guess =
      (camera_parameters.P().topLeftCorner<3, 3>().inverse() *
       Eigen::Vector3d(pixel_location[0], pixel_location[1], 1.0))
          .head<2>();

  std::pair<const Eigen::Vector2d&, const InputCameraParameters&> data_pair =
      std::make_pair(pixel_location, camera_parameters);

  nlopt::opt opt(nlopt::LN_NELDERMEAD, 2);

  opt.set_min_objective(PointToBearing::bearingProjectionError,
                        static_cast<void*>(&data_pair));
  opt.set_xtol_rel(1e-3);

  std::vector<double> values = {inital_guess.x(), inital_guess.y()};
  double minf;
  nlopt::result result = opt.optimize(values, minf);

  *bearing = Eigen::Vector3d(values[0], values[1], 1.0);
  bearing->normalize();
}

double PointToBearing::bearingProjectionError(const std::vector<double>& values,
                              std::vector<double>& grad, void* data) {
  // split out inputs
  const Eigen::Vector2d pixel_location(values[0], values[1]);
  const std::pair<const Eigen::Vector2d&, const InputCameraParameters&>
      data_pair = *static_cast<
          std::pair<const Eigen::Vector2d&, const InputCameraParameters&>*>(
          data);
  const Eigen::Vector2d& distorted_pixel_location = data_pair.first;
  const InputCameraParameters& camera_parameters = data_pair.second;

  Eigen::Matrix<double, 3, 4> P_bearing;
  P_bearing.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
  P_bearing.topRightCorner<3, 1>() = Eigen::Vector3d::Zero();

  Eigen::Vector2d estimated_distorted_pixel_location;
  Undistorter::distortPixel(camera_parameters.K(), camera_parameters.R(),
                            P_bearing, camera_parameters.distortionModel(),
                            camera_parameters.D(), pixel_location,
                            &estimated_distorted_pixel_location);

  return (estimated_distorted_pixel_location - distorted_pixel_location).norm();
}

void PointToBearing::cameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr& camera_info) {
  try {
    camera_parameters_ptr_ =
        std::make_shared<InputCameraParameters>(*camera_info);
  } catch (std::runtime_error e) {
    ROS_ERROR("%s", e.what());
  }
}
}
