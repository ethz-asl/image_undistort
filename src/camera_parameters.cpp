#include "image_undistort/camera_parameters.h"
#include "image_undistort/undistorter.h"

namespace image_undistort {

BaseCameraParameters::BaseCameraParameters(const ros::NodeHandle& nh,
                                           const std::string& camera_namespace,
                                           const bool invert_T) {
  ROS_INFO("Loading camera parameters");

  XmlRpc::XmlRpcValue K_in;
  bool K_loaded = false;
  if (nh.getParam(camera_namespace + "/K", K_in)) {
    xmlRpcToMatrix(K_in, &K_);
    K_loaded = true;
  }

  std::vector<double> intrinsics_in;
  if (nh.getParam(camera_namespace + "/intrinsics", intrinsics_in)) {
    if (K_loaded) {
      ROS_WARN(
          "Both K and intrinsics vector given, ignoring intrinsics "
          "vector");
    } else if (intrinsics_in.size() < 4) {
      throw std::runtime_error(
          "Intrinsics vector must have at least 4 values (Fx,Fy,Cx,Cy)");
    }

    K_ = Eigen::Matrix3d::Identity();
    K_(0, 0) = intrinsics_in[intrinsics_in.size() - 4];
    K_(1, 1) = intrinsics_in[intrinsics_in.size() - 3];
    K_(0, 2) = intrinsics_in[intrinsics_in.size() - 2];
    K_(1, 2) = intrinsics_in[intrinsics_in.size() - 1];
  } else if (!K_loaded) {
    throw std::runtime_error("Could not find K or camera intrinsics vector");
  }

  std::vector<double> resolution_in;
  if (nh.getParam(camera_namespace + "/resolution", resolution_in)) {
    if (resolution_in.size() != 2) {
      throw std::runtime_error("Resolution must have exactly 2 values (x,y)");
    }
    resolution_.width = resolution_in[0];
    resolution_.height = resolution_in[1];
  } else {
    throw std::runtime_error("Could not find camera resolution");
  }

  XmlRpc::XmlRpcValue T_in;
  bool T_loaded = false;
  if (nh.getParam(camera_namespace + "/T_cn_cnm1", T_in) ||
      nh.getParam(camera_namespace + "/T", T_in)) {
    xmlRpcToMatrix(T_in, &T_);
    T_loaded = true;
  } else {
    T_ = Eigen::Matrix4d::Identity();
  }

  if (invert_T) {
    T_ = T_.inverse();
  }

  XmlRpc::XmlRpcValue P_in;
  if (nh.getParam(camera_namespace + "/P", P_in)) {
    xmlRpcToMatrix(P_in, &P_);

    if (!T_loaded) {
      T_.topLeftCorner<3, 3>() = K_.inverse() * P_.topLeftCorner<3, 3>();
      T_.topRightCorner<3, 1>() = K_.inverse() * P_.topRightCorner<3, 1>();
      T_(3, 3) = 1;
    } else if (!P_.isApprox((Eigen::Matrix<double, 3, 4>() << K_,
                             Eigen::Vector3d::Constant(0))
                                .finished() *
                            T_)) {
      ROS_WARN_ONCE(
          "For given K, T and P ([K,[0;0;0]]*T != P), replacing K with "
          "corrected value");
      K_ = P_.topLeftCorner<3, 3>() * T_.topLeftCorner<3, 3>().inverse();
    }
  } else {
    P_ = (Eigen::Matrix<double, 3, 4>() << K_, Eigen::Vector3d::Constant(0))
             .finished() *
         T_;
  }
}

BaseCameraParameters::BaseCameraParameters(
    const sensor_msgs::CameraInfo& camera_info) {
  resolution_.height = camera_info.height;
  resolution_.width = camera_info.width;

  K_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
      camera_info.K.data());

  T_ = Eigen::Matrix4d::Identity();
  T_.topLeftCorner<3, 3>() =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          camera_info.R.data());

  P_ = Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(
      camera_info.P.data());

  T_.topRightCorner<3, 1>() = K_.inverse() * P_.topRightCorner<3, 1>();

  if (!P_.topLeftCorner<3, 3>().isApprox(K_ * T_.topLeftCorner<3, 3>())) {
    ROS_WARN_ONCE(
        "For given K, T and P ([K,[0;0;0]]*T != P), replacing K with corrected "
        "value");
    K_ = P_.topLeftCorner<3, 3>() * T_.topLeftCorner<3, 3>().inverse();
  }
}

BaseCameraParameters::BaseCameraParameters(
    const cv::Size& resolution_in, const Eigen::Matrix<double, 4, 4>& T_in,
    const Eigen::Matrix<double, 3, 3>& K_in)
    : resolution_(resolution_in),
      T_(T_in),
      P_((Eigen::Matrix<double, 3, 4>() << K_in, Eigen::Vector3d::Constant(0))
             .finished() *
         T_in),
      K_(K_in) {}

const cv::Size& BaseCameraParameters::resolution() const { return resolution_; }

const Eigen::Matrix<double, 4, 4>& BaseCameraParameters::T() const {
  return T_;
}
const Eigen::Ref<const Eigen::Matrix<double, 3, 3>> BaseCameraParameters::R()
    const {
  return T_.topLeftCorner<3, 3>();
}
const Eigen::Ref<const Eigen::Matrix<double, 3, 1>> BaseCameraParameters::p()
    const {
  return T_.topRightCorner<3, 1>();
}

const Eigen::Matrix<double, 3, 4>& BaseCameraParameters::P() const {
  return P_;
}
const Eigen::Matrix<double, 3, 3>& BaseCameraParameters::K() const {
  return K_;
}

bool BaseCameraParameters::operator==(const BaseCameraParameters& B) const {
  return (resolution() == B.resolution()) && (T() == B.T()) && (P() == B.P()) &&
         (K() == B.K());
}

bool BaseCameraParameters::operator!=(const BaseCameraParameters& B) const {
  return !(*this == B);
}

InputCameraParameters::InputCameraParameters(
    const ros::NodeHandle& nh, const std::string& camera_namespace,
    const bool invert_T)
    : BaseCameraParameters(nh, camera_namespace, invert_T) {
  std::string distortion_model_in, camera_model_in;
  if (!nh.getParam(camera_namespace + "/distortion_model",
                   distortion_model_in) ||
      !nh.getParam(camera_namespace + "/camera_model", camera_model_in)) {
    ROS_WARN(
        "No camera and/or distortion model given, assuming pinhole-radtan");
    distortion_model_ = DistortionModel::RADTAN;
  } else {
    distortion_model_ =
        stringToDistortion(distortion_model_in, camera_model_in);
  }

  std::vector<double> intrinsics_in;
  if (nh.getParam(camera_namespace + "/intrinsics", intrinsics_in)) {
    if (intrinsics_in.size() > 4) {
      D_.push_back(intrinsics_in[0]);
    }
    if (intrinsics_in.size() > 5) {
      D_.push_back(intrinsics_in[1]);
    }
    if (intrinsics_in.size() > 6) {
      throw std::runtime_error(
          "Intrinsics vector cannot have more than 6 values");
    }
  }

  std::vector<double> D_in;
  if (nh.getParam(camera_namespace + "/distortion_coeffs", D_in)) {
    D_.insert(D_.end(), D_in.begin(), D_in.end());
  }

  if (D_.empty()) {
    ROS_WARN(
        "No distortion coefficients found, assuming images are "
        "undistorted");
  }

  // ensure D always has at least 7 elements
  while (D_.size() < 7) {
    D_.push_back(0);
  }
}

InputCameraParameters::InputCameraParameters(
    const sensor_msgs::CameraInfo& camera_info)
    : BaseCameraParameters(camera_info),
      D_(camera_info.D),
      distortion_model_(
          stringToDistortion(camera_info.distortion_model, "pinhole")) {
  // ensure D always has at least 5 elements
  while (D_.size() < 5) {
    D_.push_back(0);
  }
}

InputCameraParameters::InputCameraParameters(
    const cv::Size& resolution_in, const Eigen::Matrix<double, 4, 4>& T_in,
    const Eigen::Matrix<double, 3, 3>& K_in, const std::vector<double>& D_in,
    const DistortionModel& distortion_model)
    : BaseCameraParameters(resolution_in, T_in, K_in),
      D_(D_in),
      distortion_model_(distortion_model) {
  // ensure D always has at least 5 elements
  while (D_.size() < 5) {
    D_.push_back(0);
  }
}

const std::vector<double>& InputCameraParameters::D() const { return D_; }

const DistortionModel& InputCameraParameters::distortionModel() const {
  return distortion_model_;
}

const DistortionModel InputCameraParameters::stringToDistortion(
    const std::string& distortion_model, const std::string& camera_model) {
  std::string lower_case_distortion_model = distortion_model;
  std::string lower_case_camera_model = camera_model;

  std::transform(lower_case_distortion_model.begin(),
                 lower_case_distortion_model.end(),
                 lower_case_distortion_model.begin(), ::tolower);
  std::transform(lower_case_camera_model.begin(), lower_case_camera_model.end(),
                 lower_case_camera_model.begin(), ::tolower);

  if (lower_case_camera_model == "pinhole") {
    if (lower_case_camera_model == std::string("none")) {
      return DistortionModel::NONE;
    } else if ((lower_case_distortion_model == std::string("plumb bob")) ||
               (lower_case_distortion_model == std::string("plumb_bob")) ||
               (lower_case_distortion_model == std::string("radtan"))) {
      return DistortionModel::RADTAN;
    } else if (lower_case_distortion_model == std::string("equidistant")) {
      return DistortionModel::EQUIDISTANT;
    } else if (lower_case_distortion_model == std::string("fov")) {
      return DistortionModel::FOV;
    } else {
      throw std::runtime_error(
          "Unrecognized distortion model for pinhole camera. Valid pinhole "
          "distortion model options are 'none', 'radtan', 'Plumb Bob', "
          "'plumb_bob', "
          "'equidistant' and 'fov'.");
    }

  } else if (lower_case_camera_model == "omni") {
    if ((lower_case_distortion_model == std::string("plumb bob")) ||
        (lower_case_distortion_model == std::string("plumb_bob")) ||
        (lower_case_distortion_model == std::string("radtan"))) {
      return DistortionModel::OMNIRADTAN;
    } else if (lower_case_distortion_model == std::string("none")) {
      return DistortionModel::OMNI;
    } else {
      throw std::runtime_error(
          "Unrecognized distortion model for omni camera. Valid omni "
          "distortion model options are 'none' and 'radtan'.");
    }

  } else if ((lower_case_camera_model == std::string("double_sphere")) ||
             (lower_case_camera_model == std::string("ds"))) {
    return DistortionModel::DOUBLESPHERE;
  } else if (lower_case_camera_model == std::string("unified")) {
    return DistortionModel::UNIFIED;
  } else if ((lower_case_camera_model == std::string("extended_unified")) ||
             (lower_case_camera_model == std::string("eucm"))) {
    return DistortionModel::EXTENDEDUNIFIED;
  } else {
    throw std::runtime_error(
        "Unrecognized camera model. Valid camera models are 'pinhole', "
        "'omni', 'double_sphere', 'ds', 'unified', 'extended_unified' and "
        "'eucm'");
  }
}

bool InputCameraParameters::operator==(const InputCameraParameters& B) const {
  return (*dynamic_cast<const BaseCameraParameters*>(this) == B) &&
         (D() == B.D()) && (distortionModel() == B.distortionModel());
}

bool InputCameraParameters::operator!=(const InputCameraParameters& B) const {
  return !(*this == B);
}

// holds the camera parameters of the input camera and virtual output camera
CameraParametersPair::CameraParametersPair(
    const DistortionProcessing distortion_processing)
    : distortion_processing_(distortion_processing){};

bool CameraParametersPair::setCameraParameters(
    const ros::NodeHandle& nh, const std::string& camera_namespace,
    const CameraIO& io, const bool invert_T) {
  try {
    if (io == CameraIO::INPUT) {
      input_ptr_ = std::make_shared<InputCameraParameters>(nh, camera_namespace,
                                                           invert_T);
    } else {
      output_ptr_ = std::make_shared<OutputCameraParameters>(
          nh, camera_namespace, invert_T);
    }
    return true;
  } catch (std::runtime_error e) {
    ROS_ERROR("%s", e.what());
    return false;
  }
}

bool CameraParametersPair::setCameraParameters(
    const sensor_msgs::CameraInfo& camera_info, const CameraIO& io) {
  try {
    if (io == CameraIO::INPUT) {
      input_ptr_ = std::make_shared<InputCameraParameters>(camera_info);
    } else {
      output_ptr_ = std::make_shared<OutputCameraParameters>(camera_info);
    }
    return true;
  } catch (std::runtime_error e) {
    ROS_ERROR("%s", e.what());
    return false;
  }
}

bool CameraParametersPair::setInputCameraParameters(
    const cv::Size& resolution, const Eigen::Matrix<double, 4, 4>& T,
    const Eigen::Matrix<double, 3, 3>& K, const std::vector<double>& D,
    const DistortionModel& distortion_model) {
  try {
    input_ptr_ = std::make_shared<InputCameraParameters>(resolution, T, K, D,
                                                         distortion_model);
    return true;
  } catch (std::runtime_error e) {
    ROS_ERROR("%s", e.what());
    return false;
  }
}

bool CameraParametersPair::setOutputCameraParameters(
    const cv::Size& resolution, const Eigen::Matrix<double, 4, 4>& T,
    const Eigen::Matrix<double, 3, 3>& K) {
  try {
    output_ptr_ = std::make_shared<OutputCameraParameters>(resolution, T, K);
    return true;
  } catch (std::runtime_error e) {
    ROS_ERROR("%s", e.what());
    return false;
  }
}

bool CameraParametersPair::setOutputFromInput(const double scale) {
  if (!valid(CameraIO::INPUT)) {
    ROS_ERROR(
        "Cannot set output to same values as input, as input is not currently "
        "set");
    return false;
  } else {
    Eigen::Matrix<double, 3, 3> K = input_ptr_->K();
    K(0, 0) *= scale;
    K(1, 1) *= scale;
    setOutputCameraParameters(input_ptr_->resolution(), input_ptr_->T(), K);
    return true;
  }
}

bool CameraParametersPair::setOptimalOutputCameraParameters(
    const double scale) {
  if (!valid(CameraIO::INPUT)) {
    ROS_ERROR(
        "Optimal output camera parameters cannot be set until the input camera "
        "parameters have been given");
    return false;
  }
  cv::Size resolution_estimate(
      std::ceil(input_ptr_->resolution().width * scale),
      std::ceil(input_ptr_->resolution().height * scale));
  double focal_length =
      scale * (input_ptr_->K()(0, 0) + input_ptr_->K()(1, 1)) / 2;
  Eigen::Matrix<double, 3, 4> P = Eigen::Matrix<double, 3, 4>::Zero();
  P(0, 0) = focal_length;
  P(1, 1) = focal_length;
  P(2, 2) = 1;
  P(0, 2) = static_cast<double>(resolution_estimate.width) / 2.0;
  P(1, 2) = static_cast<double>(resolution_estimate.height) / 2.0;
  P.topRightCorner<3, 1>() = focal_length * input_ptr_->p();

  std::vector<double> D;
  if (distortion_processing_ == DistortionProcessing::UNDISTORT) {
    D = input_ptr_->D();
  } else {
    D = std::vector<double>(7, 0);
  }

  // Find the resolution of the output image
  // Thanks to weird corner cases this is way more complex then it should be.
  // The general case is even more of a nightmare, so we constrain the problem
  // such that the center of focus must be in the center of the final image.
  // As we are missing the forward projection model we iteratively estimate
  // image size assuming a linear relationship between warping and size at each
  // step
  for (size_t i = 0; i < kFocalLengthEstimationAttempts; ++i) {
    // get list of edge points to check
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
        pixel_locations;
    for (size_t v = 0; v < resolution_estimate.height; ++v) {
      pixel_locations.emplace_back(0, v);
      pixel_locations.emplace_back(resolution_estimate.width - 1, v);
    }
    for (size_t u = 1; u < (resolution_estimate.width - 1); ++u) {
      pixel_locations.emplace_back(u, 0);
      pixel_locations.emplace_back(u, resolution_estimate.height - 1);
    }

    // find extreme points
    double max_x = 0;
    double max_y = 0;
    for (Eigen::Vector2d pixel_location : pixel_locations) {
      Eigen::Vector2d distorted_pixel_location;
      Undistorter::distortPixel(input_ptr_->K(), input_ptr_->R(), P,
                                input_ptr_->distortionModel(), D,
                                pixel_location, &distorted_pixel_location);

      max_x = std::max(
          max_x,
          std::abs(static_cast<double>(input_ptr_->resolution().width) / 2.0 -
                   distorted_pixel_location.x()));
      max_y = std::max(
          max_y,
          std::abs(static_cast<double>(input_ptr_->resolution().height) / 2.0 -
                   distorted_pixel_location.y()));
    }

    // change resolution estimate so that extreme points lie on edges (under
    // the aforementioned linear assumption)
    cv::Size resolution_update;
    resolution_update.width = std::floor(
        static_cast<double>(resolution_estimate.width) *
        std::abs(static_cast<double>(input_ptr_->resolution().width) - max_x) /
        (static_cast<double>(input_ptr_->resolution().width) / 2.0));
    resolution_update.height = std::floor(
        static_cast<double>(resolution_estimate.height) *
        std::abs(static_cast<double>(input_ptr_->resolution().height) - max_y) /
        (static_cast<double>(input_ptr_->resolution().height) / 2.0));

    if (resolution_update == resolution_estimate) {
      break;
    } else {
      resolution_estimate = resolution_update;
      P(0, 2) = static_cast<double>(resolution_estimate.width) / 2.0;
      P(1, 2) = static_cast<double>(resolution_estimate.height) / 2.0;
    }
  }

  // create final camera parameters
  Eigen::Matrix3d K = P.topLeftCorner<3, 3>();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topRightCorner<3, 1>() = input_ptr_->p();

  return setOutputCameraParameters(resolution_estimate, T, K);
}

void CameraParametersPair::generateCameraInfoMessage(
    const CameraIO& io, sensor_msgs::CameraInfo* camera_info) const {
  if (!valid()) {
    throw std::runtime_error(
        "Attempted to get output camera_info before a valid input and output "
        "has been set");
  } else {
    std::shared_ptr<BaseCameraParameters> camera_parameters_ptr;
    if (io == CameraIO::INPUT) {
      camera_parameters_ptr = input_ptr_;
    } else {
      camera_parameters_ptr = output_ptr_;
    }

    camera_info->height = camera_parameters_ptr->resolution().height;
    camera_info->width = camera_parameters_ptr->resolution().width;

    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        camera_info->K.data()) = camera_parameters_ptr->K();

    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        camera_info->R.data()) = camera_parameters_ptr->R();

    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(
        camera_info->P.data()) = camera_parameters_ptr->P();

    if (io == CameraIO::OUTPUT ||
        distortion_processing_ == DistortionProcessing::UNDISTORT) {
      camera_info->D = std::vector<double>(5, 0);
    } else {
      camera_info->D = input_ptr_->D();
    }
    if (input_ptr_->distortionModel() == DistortionModel::RADTAN) {
      camera_info->distortion_model = "radtan";
    } else {
      camera_info->distortion_model = "equidistant";
    }
  }
}

const DistortionProcessing& CameraParametersPair::distortionProcessing() const {
  return distortion_processing_;
}

const std::shared_ptr<InputCameraParameters>&
CameraParametersPair::getInputPtr() const {
  return input_ptr_;
}
const std::shared_ptr<OutputCameraParameters>&
CameraParametersPair::getOutputPtr() const {
  return output_ptr_;
}

bool CameraParametersPair::valid() const {
  return (input_ptr_ != nullptr) && (output_ptr_ != nullptr);
}

bool CameraParametersPair::valid(const CameraIO& io) const {
  if (io == CameraIO::INPUT) {
    return input_ptr_ != nullptr;
  } else {
    return output_ptr_ != nullptr;
  }
}

bool CameraParametersPair::operator==(const CameraParametersPair& B) const {
  return *getInputPtr() == *B.getInputPtr() &&
         (*getOutputPtr() == *B.getOutputPtr());
}

bool CameraParametersPair::operator!=(const CameraParametersPair& B) const {
  return !(*this == B);
}

StereoCameraParameters::StereoCameraParameters(const double scale)
    : scale_(scale){};

bool StereoCameraParameters::setInputCameraParameters(
    const ros::NodeHandle& nh, const std::string& camera_namespace,
    const CameraSide& side, const bool invert_T) {
  bool success;
  if (side == CameraSide::FIRST) {
    success = first_.setCameraParameters(nh, camera_namespace, CameraIO::INPUT,
                                         invert_T);
  } else {
    success = second_.setCameraParameters(nh, camera_namespace, CameraIO::INPUT,
                                          invert_T);
  }
  if (valid(CameraSide::FIRST, CameraIO::INPUT) &&
      valid(CameraSide::SECOND, CameraIO::INPUT)) {
    generateRectificationParameters();
  }
  return success;
}

bool StereoCameraParameters::setInputCameraParameters(
    const sensor_msgs::CameraInfo& camera_info, const CameraSide& side) {
  try {
    if (side == CameraSide::FIRST) {
      first_.setCameraParameters(camera_info, CameraIO::INPUT);
    } else {
      second_.setCameraParameters(camera_info, CameraIO::INPUT);
    }
    if (valid(CameraSide::FIRST, CameraIO::INPUT) &&
        valid(CameraSide::SECOND, CameraIO::INPUT)) {
      generateRectificationParameters();
    }
    return true;
  } catch (std::runtime_error e) {
    ROS_ERROR("%s", e.what());
    return false;
  }
}

bool StereoCameraParameters::setInputCameraParameters(
    const cv::Size& resolution, const Eigen::Matrix<double, 4, 4>& T,
    const Eigen::Matrix<double, 3, 3>& K, const std::vector<double>& D,
    const DistortionModel& distortion_model, const CameraSide& side) {
  try {
    if (side == CameraSide::FIRST) {
      first_.setInputCameraParameters(resolution, T, K, D, distortion_model);
    } else {
      second_.setInputCameraParameters(resolution, T, K, D, distortion_model);
    }
    if (valid(CameraSide::FIRST, CameraIO::INPUT) &&
        valid(CameraSide::SECOND, CameraIO::INPUT)) {
      generateRectificationParameters();
    }
    return true;
  } catch (std::runtime_error e) {
    ROS_ERROR("%s", e.what());
    return false;
  }
}

void StereoCameraParameters::generateCameraInfoMessage(
    const CameraSide& side, const CameraIO& io,
    sensor_msgs::CameraInfo* camera_info) const {
  if (side == CameraSide::FIRST) {
    first_.generateCameraInfoMessage(io, camera_info);
  } else {
    second_.generateCameraInfoMessage(io, camera_info);
  }
}

bool StereoCameraParameters::valid() const {
  return first_.valid() && second_.valid();
}
bool StereoCameraParameters::valid(const CameraSide& side,
                                   const CameraIO& io) const {
  if (side == CameraSide::FIRST) {
    return first_.valid(io);
  } else {
    return second_.valid(io);
  }
}

bool StereoCameraParameters::generateRectificationParameters() {
  if (first_.getInputPtr()->p().isApprox(second_.getInputPtr()->p())) {
    ROS_ERROR(
        "Stereo rectification cannot be performed on cameras with a baseline "
        "of zero");
    return false;
  }

  // twist inputs to align on x axis
  Eigen::Vector3d x = first_.getInputPtr()->p() - second_.getInputPtr()->p();
  Eigen::Vector3d y = first_.getInputPtr()->R().col(2).cross(x);
  Eigen::Vector3d z = x.cross(y);

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() << x.normalized(), y.normalized(), z.normalized();

  // took wrong camera as left (redo other way round)
  if (T(0, 0) < 0) {
    x = second_.getInputPtr()->p() - first_.getInputPtr()->p();
    y = second_.getInputPtr()->R().col(2).cross(x);
    z = x.cross(y);
    T.topLeftCorner<3, 3>() << x.normalized(), y.normalized(), z.normalized();
  }

  first_.setInputCameraParameters(
      first_.getInputPtr()->resolution(),
      T.inverse() * first_.getInputPtr()->T(), first_.getInputPtr()->K(),
      first_.getInputPtr()->D(), first_.getInputPtr()->distortionModel());
  second_.setInputCameraParameters(
      second_.getInputPtr()->resolution(),
      T.inverse() * second_.getInputPtr()->T(), second_.getInputPtr()->K(),
      second_.getInputPtr()->D(), second_.getInputPtr()->distortionModel());

  // set individual outputs
  if (!first_.setOptimalOutputCameraParameters(scale_) ||
      !second_.setOptimalOutputCameraParameters(scale_)) {
    ROS_ERROR("Automatic generation of stereo output parameters failed");
    return false;
  }

  // grab most conservative values
  cv::Size resolution(std::min(first_.getOutputPtr()->resolution().width,
                               second_.getOutputPtr()->resolution().width),
                      std::min(first_.getOutputPtr()->resolution().height,
                               second_.getOutputPtr()->resolution().height));

  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
  K(0, 0) = std::max(first_.getOutputPtr()->K()(0, 0),
                     second_.getOutputPtr()->K()(0, 0));
  K(1, 1) = K(0, 0);
  K(0, 2) = static_cast<double>(resolution.width) / 2.0;
  K(1, 2) = static_cast<double>(resolution.height) / 2.0;
  K(2, 2) = 1;

  // set the new consistent outputs
  if (!first_.setOutputCameraParameters(resolution, first_.getOutputPtr()->T(),
                                        K) ||
      !second_.setOutputCameraParameters(resolution,
                                         second_.getOutputPtr()->T(), K)) {
    ROS_ERROR("Automatic generation of stereo output parameters failed");
    return false;
  }

  return true;
}

const CameraParametersPair& StereoCameraParameters::getFirst() const {
  return first_;
}

const CameraParametersPair& StereoCameraParameters::getSecond() const {
  return second_;
}
}  // namespace image_undistort