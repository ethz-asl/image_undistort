#ifndef IMAGE_UNDISTORT_CAMERA_PARAMETERS_H
#define IMAGE_UNDISTORT_CAMERA_PARAMETERS_H

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>

namespace image_undistort {

enum class CameraSide { FIRST, SECOND };
enum class CameraIO { INPUT, OUTPUT };
enum class DistortionModel {
  NONE,
  RADTAN,
  EQUIDISTANT,
  FOV,
  OMNI,
  OMNIRADTAN,
  DOUBLESPHERE,
  UNIFIED,
  EXTENDEDUNIFIED
};
enum class DistortionProcessing { UNDISTORT, PRESERVE };

// holds basic properties of a camera
class BaseCameraParameters {
 public:
  BaseCameraParameters(const ros::NodeHandle& nh,
                       const std::string& camera_namespace,
                       const bool invert_T);

  BaseCameraParameters(const sensor_msgs::CameraInfo& camera_info);

  BaseCameraParameters(const cv::Size& resolution_in,
                       const Eigen::Matrix<double, 4, 4>& T_in,
                       const Eigen::Matrix<double, 3, 3>& K_in);

  const cv::Size& resolution() const;  // get image size

  const Eigen::Matrix<double, 4, 4>& T() const;  // get transformation matrix
  const Eigen::Ref<const Eigen::Matrix<double, 3, 3>> R()
      const;  // get rotation matrix
  const Eigen::Ref<const Eigen::Matrix<double, 3, 1>> p()
      const;  // get position vector

  const Eigen::Matrix<double, 3, 4>& P() const;  // get projection matrix
  const Eigen::Matrix<double, 3, 3>& K() const;  // get camera matrix

  bool operator==(const BaseCameraParameters& B) const;
  bool operator!=(const BaseCameraParameters& B) const;

 private:
  template <typename Derived>
  static void xmlRpcToMatrix(const XmlRpc::XmlRpcValue& const_input,
                             Eigen::MatrixBase<Derived>* output) {
    // A local copy is required as the methods that allow you to access the
    // XmlRpc values as doubles are not const and so cannot be used with the
    // const ref
    XmlRpc::XmlRpcValue input = const_input;

    if (input.size() != output->rows()) {
      throw std::runtime_error("Loaded matrix has incorrect number of rows");
    }
    for (size_t i = 0; i < output->rows(); ++i) {
      if (input[i].size() != output->cols()) {
        throw std::runtime_error(
            "Loaded matrix has incorrect number of columns");
      }
      for (size_t j = 0; j < output->cols(); ++j) {
        output->coeffRef(i, j) = input[i][j];
      }
    }
  }

  cv::Size resolution_;
  Eigen::Matrix<double, 4, 4> T_;
  Eigen::Matrix<double, 3, 4> P_;
  Eigen::Matrix<double, 3, 3> K_;
};

// basic camera properties + distortion parameters
class InputCameraParameters : public BaseCameraParameters {
 public:
  InputCameraParameters(const ros::NodeHandle& nh,
                        const std::string& camera_namespace,
                        const bool invert_T = false);

  InputCameraParameters(const sensor_msgs::CameraInfo& camera_info);

  InputCameraParameters(const cv::Size& resolution_in,
                        const Eigen::Matrix<double, 4, 4>& T_in,
                        const Eigen::Matrix<double, 3, 3>& K_in,
                        const std::vector<double>& D_in,
                        const DistortionModel& distortion_model);

  const std::vector<double>& D() const;  // get distortion vector
  const DistortionModel& distortionModel() const;

  bool operator==(const InputCameraParameters& B) const;
  bool operator!=(const InputCameraParameters& B) const;

 private:
  static const DistortionModel stringToDistortion(
      const std::string& distortion_model, const std::string& camera_model);

  std::vector<double> D_;
  DistortionModel distortion_model_;
};

// basic camera properties + anything special to output cameras (currently
// nothing)
class OutputCameraParameters : public BaseCameraParameters {
 public:
  using BaseCameraParameters::BaseCameraParameters;
};

// holds the camera parameters of the input camera and virtual output camera
class CameraParametersPair {
 public:
  CameraParametersPair(const DistortionProcessing distortion_processing =
                           DistortionProcessing::UNDISTORT);

  bool setCameraParameters(const ros::NodeHandle& nh,
                           const std::string& camera_namespace,
                           const CameraIO& io, const bool invert_T = false);

  bool setCameraParameters(const sensor_msgs::CameraInfo& camera_info,
                           const CameraIO& io);

  bool setInputCameraParameters(const cv::Size& resolution,
                                const Eigen::Matrix<double, 4, 4>& T,
                                const Eigen::Matrix<double, 3, 3>& K,
                                const std::vector<double>& D,
                                const DistortionModel& distortion_model);

  bool setOutputCameraParameters(const cv::Size& resolution,
                                 const Eigen::Matrix<double, 4, 4>& T,
                                 const Eigen::Matrix<double, 3, 3>& K);

  bool setOutputFromInput(const double scale);

  bool setOptimalOutputCameraParameters(const double scale);

  const DistortionProcessing& distortionProcessing() const;

  void generateCameraInfoMessage(const CameraIO& io,
                                 sensor_msgs::CameraInfo* camera_info) const;

  const std::shared_ptr<InputCameraParameters>& getInputPtr() const;
  const std::shared_ptr<OutputCameraParameters>& getOutputPtr() const;

  bool valid() const;
  bool valid(const CameraIO& io) const;

  bool operator==(const CameraParametersPair& B) const;
  bool operator!=(const CameraParametersPair& B) const;

 private:
  std::shared_ptr<InputCameraParameters> input_ptr_;
  std::shared_ptr<OutputCameraParameters> output_ptr_;

  DistortionProcessing distortion_processing_;

  static constexpr double kFocalLengthEstimationAttempts = 100;
};

// holds the camera parameters of the first and second camera and uses them to
// generate virtual output cameras with properties that will produce correctly
// rectified images
class StereoCameraParameters {
 public:
  StereoCameraParameters(const double scale = 1.0);

  bool setInputCameraParameters(const ros::NodeHandle& nh,
                                const std::string& camera_namespace,
                                const CameraSide& side, const bool invert_T);

  bool setInputCameraParameters(const sensor_msgs::CameraInfo& camera_info,
                                const CameraSide& side);

  bool setInputCameraParameters(const cv::Size& resolution,
                                const Eigen::Matrix<double, 4, 4>& T,
                                const Eigen::Matrix<double, 3, 3>& K,
                                const std::vector<double>& D,
                                const DistortionModel& distortion_model,
                                const CameraSide& side);

  void generateCameraInfoMessage(const CameraSide& side, const CameraIO& io,
                                 sensor_msgs::CameraInfo* camera_info) const;

  const CameraParametersPair& getFirst() const;
  const CameraParametersPair& getSecond() const;

  bool valid() const;
  bool valid(const CameraSide& side, const CameraIO& io) const;

 private:
  bool generateRectificationParameters();

  double scale_;
  CameraParametersPair first_;
  CameraParametersPair second_;
};
}  // namespace image_undistort
#endif  // CAMERA_PARAMETERS_H
