#ifndef CAMERA_PARAMETERS_H
#define CAMERA_PARAMETERS_H

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>

#include <Eigen/Eigen>

// holds basic properties of a camera
class BaseCameraParameters {
 public:
  BaseCameraParameters(const ros::NodeHandle& nh,
                       const std::string& camera_namespace);

  BaseCameraParameters(const sensor_msgs::CameraInfo& camera_info);

  BaseCameraParameters(const cv::Size& resolution_in,
                       const Eigen::Matrix<double, 4, 4>& T_in,
                       const Eigen::Matrix<double, 3, 4>& K_in);

  const cv::Size& resolution() const;  // get image size

  const Eigen::Matrix<double, 4, 4>& T() const;  // get transformation matrix
  const Eigen::Matrix<double, 3, 3>& R() const;  // get rotation matrix
  const Eigen::Matrix<double, 3, 1>& p() const;  // get position vector

  const Eigen::Matrix<double, 3, 4>& P() const;  // get projection matrix
  const Eigen::Matrix<double, 3, 3>& K() const;  // get camera matrix

  bool operator==(const BaseCameraParameters& B) const;

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
    return true;
  }

  cv::Size resolution_;
  Eigen::Matrix<double, 4, 4> T_;
  Eigen::Matrix<double, 3, 3> P_;
  Eigen::Matrix<double, 3, 3> K_;
};

// basic camera properties + distortion parameters
class InputCameraParameters : public BaseCameraParameters {
 public:
  InputCameraParameters(const ros::NodeHandle& nh,
                        const std::string& camera_namespace);

  InputCameraParameters(const sensor_msgs::CameraInfo& camera_info);

  InputCameraParameters(const cv::Size& resolution_in,
                        const Eigen::Matrix<double, 4, 4>& T_in,
                        const Eigen::Matrix<double, 3, 4>& K_in,
                        const std::vector<double>& D_in,
                        const bool radtan_distortion);

  const std::vector<double>& D() const;       // get distortion vector
  const bool usingRadtanDistortion() const;  // gets if using radtan distortion

  bool operator==(const InputCameraParameters& B) const;

 private:
  static bool is_radtan_distortion(const std::string& distortion_model);

  std::vector<double> D_;
  bool radtan_distortion_;
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
  CameraParametersPair(const bool undistort = true);

  bool setCameraParameters(const ros::NodeHandle& nh,
                           const std::string& camera_namespace,
                           bool updating_input_camera);

  bool setCameraParameters(const sensor_msgs::CameraInfo& camera_info,
                           bool updating_input_camera);

  bool setInputCameraParameters(const cv::Size& resolution,
                                const Eigen::Matrix<double, 4, 4>& T,
                                const Eigen::Matrix<double, 3, 4>& K,
                                const std::vector<double>& D,
                                const bool radtan_distortion);

  bool setOutputCameraParameters(const cv::Size& resolution,
                                 const Eigen::Matrix<double, 4, 4>& T,
                                 const Eigen::Matrix<double, 3, 4>& K);

  bool undistort() const;  // if the camera output will be undistorted

  void generateOutputCameraInfoMessage(
      sensor_msgs::CameraInfo* camera_info) const;

  const std::shared_ptr<InputCameraParameters>& getInput() const;
  const std::shared_ptr<OutputCameraParameters>& getOutput() const;

  bool valid() const;
  bool valid(const bool check_input_camera) const;

  bool operator==(const CameraParametersPair& B) const;

 private:
  std::shared_ptr<InputCameraParameters> input_;
  std::shared_ptr<OutputCameraParameters> output_;

  bool undistort_;
};

// holds the camera parameters of the left and right camera and uses them to
// generate virtual output cameras with properties that will produce correctly
// rectified images
class StereoCameraParameters {
 public:
  bool setInputCameraParameters(const ros::NodeHandle& nh,
                                const std::string& camera_namespace,
                                bool updating_left_camera);

  bool setInputCameraParameters(const sensor_msgs::CameraInfo& camera_info,
                                bool updating_left_camera);

  bool setInputCameraParameters(const cv::Size& resolution,
                                const Eigen::Matrix<double, 4, 4>& T,
                                const Eigen::Matrix<double, 3, 4>& P,
                                const std::vector<double>& D,
                                const bool radtan_distortion,
                                const bool updating_left_camera);

  void generateOutputCameraInfoMessage(
      const bool get_left_camera_info,
      sensor_msgs::CameraInfo* camera_info) const;

  bool valid() const;
  bool valid(const bool left, const bool input) const;

 private:
  CameraParametersPair left_;
  CameraParametersPair right_;
};

#endif  // CAMERA_PARAMETERS_H