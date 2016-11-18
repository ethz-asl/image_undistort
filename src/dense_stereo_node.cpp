#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "dense_stereo_node");

  nodelet::Loader manager(false);  // Don't bring up the manager ROS API
  nodelet::V_string nargv;

  ros::NodeHandle nh_private("~");
  std::string left_camera_name, right_camera_name, output_frame;
  nh_private.param("left_camera_namespace", left_camera_name,
                   std::string("left"));
  nh_private.param("right_camera_namespace", right_camera_name,
                   std::string("right"));
  nh_private.param("output_frame", output_frame,
                   std::string("rect/") + left_camera_name);
  int frame_skip;
  nh_private.param("process_every_nth_frame", frame_skip, 1);
  int queue_size;
  nh_private.param("queue_size_preprocessing", queue_size, 1);

  // STEREO INFO NODELET
  nodelet::M_string stereo_info_remap;

  stereo_info_remap["raw/left/image"] =
      ros::names::resolve(std::string("raw/") + left_camera_name + "/image");
  stereo_info_remap["raw/right/image"] =
      ros::names::resolve(std::string("raw/") + right_camera_name + "/image");

  stereo_info_remap["raw/left/camera_info"] = ros::names::resolve(
      std::string("raw/") + left_camera_name + "/camera_info");
  stereo_info_remap["raw/right/camera_info"] = ros::names::resolve(
      std::string("raw/") + right_camera_name + "/camera_info");

  stereo_info_remap["rect/left/camera_info"] = ros::names::resolve(
      std::string("rect/") + left_camera_name + "/camera_info");
  stereo_info_remap["rect/right/camera_info"] = ros::names::resolve(
      std::string("rect/") + right_camera_name + "/camera_info");

  std::string stereo_info_name = ros::this_node::getName();  // leave in node
                                                             // namespace to get
                                                             // input parameters
  manager.load(stereo_info_name, "image_undistort/StereoInfoNodelet",
               stereo_info_remap, nargv);
  ROS_INFO_STREAM("Started " << stereo_info_name << "_stereo_info"
                             << " nodelet.");

  // LEFT CAMERA NODELET
  XmlRpc::XmlRpcValue left_params;
  left_params["output_camera_info_source"] = "camera_info";
  left_params["process_every_nth_frame"] = frame_skip;
  left_params["output_frame"] = output_frame;
  left_params["queue_size"] = queue_size;

  nodelet::M_string left_remap;
  left_remap["input/image"] =
      ros::names::resolve(std::string("raw/") + left_camera_name + "/image");
  left_remap[left_camera_name + "/camera_info"] = ros::names::resolve(
      std::string("raw/") + left_camera_name + "/camera_info");
  left_remap["output/camera_info"] = ros::names::resolve(
      std::string("rect/") + left_camera_name + "/camera_info");
  left_remap["output/image"] =
      ros::names::resolve(std::string("rect/") + left_camera_name + "/image");

  std::string left_name = ros::this_node::getName() + "_left";
  ros::param::set(left_name, left_params);
  manager.load(left_name, "image_undistort/ImageUndistortNodelet", left_remap,
               nargv);
  ROS_INFO_STREAM("Started " << left_name << " nodelet.");

  // RIGHT CAMERA NODELET
  XmlRpc::XmlRpcValue right_params;
  right_params["output_camera_info_source"] = "camera_info";
  right_params["process_every_nth_frame"] = frame_skip;
  right_params["publish_tf"] = false;
  right_params["output_frame"] = output_frame;
  right_params["queue_size"] = queue_size;

  nodelet::M_string right_remap;
  right_remap["input/image"] =
      ros::names::resolve(std::string("raw/") + right_camera_name + "/image");
  right_remap[right_camera_name + "/camera_info"] = ros::names::resolve(
      std::string("raw/") + right_camera_name + "/camera_info");
  right_remap["output/camera_info"] = ros::names::resolve(
      std::string("rect/") + right_camera_name + "/camera_info");
  right_remap["output/image"] =
      ros::names::resolve(std::string("rect/") + right_camera_name + "/image");

  std::string right_name = ros::this_node::getName() + "_right";
  ros::param::set(right_name, right_params);
  manager.load(right_name, "image_undistort/ImageUndistortNodelet", right_remap,
               nargv);
  ROS_INFO_STREAM("Started " << right_name << " nodelet.");

  // DISPARITY NODELET
  XmlRpc::XmlRpcValue disparity_params;
  disparity_params["approximate_sync"] = true;
  disparity_params["queue_size"] = 10;

  nodelet::M_string disparity_remap;
  disparity_remap["left/image_rect"] =
      ros::names::resolve(std::string("rect/") + left_camera_name + "/image");
  disparity_remap["right/image_rect"] =
      ros::names::resolve(std::string("rect/") + right_camera_name + "/image");
  disparity_remap["left/camera_info"] = ros::names::resolve(
      std::string("rect/") + left_camera_name + "/camera_info");
  disparity_remap["right/camera_info"] = ros::names::resolve(
      std::string("rect/") + right_camera_name + "/camera_info");

  std::string disparity_name = ros::this_node::getName() + "_disparity";
  ros::param::set(disparity_name, disparity_params);
  manager.load(disparity_name, "stereo_image_proc/disparity", disparity_remap,
               nargv);
  ROS_INFO_STREAM("Started " << disparity_name << " nodelet.");

  // POINTCLOUD NODELET
  XmlRpc::XmlRpcValue pointcloud_params;
  pointcloud_params["approximate_sync"] = true;

  nodelet::M_string pointcloud_remap;
  pointcloud_remap["left/camera_info"] = ros::names::resolve(
      std::string("rect/") + left_camera_name + "/camera_info");
  pointcloud_remap["right/camera_info"] = ros::names::resolve(
      std::string("rect/") + right_camera_name + "/camera_info");
  pointcloud_remap["left/image_rect_color"] =
      ros::names::resolve(std::string("rect/") + left_camera_name + "/image");

  std::string pointcloud_name = ros::this_node::getName() + "_pointcloud";
  ros::param::set(pointcloud_name, pointcloud_params);
  manager.load(pointcloud_name, "stereo_image_proc/point_cloud2",
               pointcloud_remap, nargv);
  ROS_INFO_STREAM("Started " << pointcloud_name << " nodelet.");

  ros::spin();
  return 0;
}