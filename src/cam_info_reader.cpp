#include <image_undistort/cam_info_reader.h>

CamInfoReader::CamInfoReader(const ros::NodeHandle& nh,
                             const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh) {
  cam_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("cam_info", 0, true);

  if (!loadCamParams()) {
    ROS_FATAL("Encounted errors that cannot be overcome, exiting");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  cam_info_pub_.publish(cam_info_);
}

bool CamInfoReader::loadCamParams() {
  ROS_INFO("Loading camera parameters");

  std::string cam_ns;
  if (!private_nh_.getParam("cam_ns", cam_ns)) {
    ROS_INFO("Could not find camera namespace, using global");
    cam_ns = "";
  }

  std::vector<double> intrinsics_in;
  if (!nh_.getParam(cam_ns + "/intrinsics", intrinsics_in)) {
    ROS_FATAL("Could not find camera intrinsics");
    return false;
  }
  if (intrinsics_in.size() != 4) {
    ROS_FATAL("Intrinsics must have exactly 4 values (Fx,Fy,Cx,Cy)");
    return false;
  }

  cam_info_.K[0] = intrinsics_in[0];
  cam_info_.K[1] = 0.0;
  cam_info_.K[2] = 0.0;
  cam_info_.K[3] = 0.0;
  cam_info_.K[4] = intrinsics_in[1];
  cam_info_.K[5] = 0.0;
  cam_info_.K[6] = intrinsics_in[2];
  cam_info_.K[7] = intrinsics_in[3];
  cam_info_.K[8] = 1.0;

  std::vector<double> resolution_in;
  if (!nh_.getParam(cam_ns + "/resolution", resolution_in)) {
    ROS_FATAL("Could not find camera resolution");
    return false;
  }
  if (resolution_in.size() != 2) {
    ROS_FATAL("Resolution must have exactly 2 values (x,y)");
    return false;
  }
  cam_info_.width = resolution_in[0];
  cam_info_.height = resolution_in[1];

  if (!nh_.getParam(cam_ns + "/distortion_model", cam_info_.distortion_model)) {
    ROS_WARN("No distortion model given, assuming radtan");
  }

  if (!nh_.getParam(cam_ns + "/distortion_coeffs", cam_info_.D)) {
    ROS_WARN(
        "No distortion coefficients found, assuming images are undistorted");
    cam_info_.D = std::vector<double>(0, 5);
  }

  // ensure D always has at least 5 elements
  while (cam_info_.D.size() < 5) {
    cam_info_.D.push_back(0);
  }

  // assume no rotation
  cam_info_.R[0] = 1.0;
  cam_info_.R[1] = 0.0;
  cam_info_.R[2] = 0.0;
  cam_info_.R[3] = 1.0;
  cam_info_.R[4] = 0.0;
  cam_info_.R[5] = 0.0;
  cam_info_.R[6] = 1.0;
  cam_info_.R[7] = 0.0;
  cam_info_.R[8] = 0.0;

  // set projection matrix to the same as K
  cam_info_.P[0] = cam_info_.K[0];
  cam_info_.P[1] = cam_info_.K[1];
  cam_info_.P[2] = cam_info_.K[2];
  cam_info_.P[3] = cam_info_.K[3];
  cam_info_.P[4] = cam_info_.K[4];
  cam_info_.P[5] = cam_info_.K[5];
  cam_info_.P[6] = cam_info_.K[6];
  cam_info_.P[7] = cam_info_.K[7];
  cam_info_.P[8] = cam_info_.K[8];
  cam_info_.P[9] = 0.0;
  cam_info_.P[10] = 0.0;
  cam_info_.P[11] = 0.0;

  return true;
}