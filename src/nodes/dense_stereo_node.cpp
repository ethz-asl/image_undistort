#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "dense_stereo_node");

  nodelet::Loader manager(false);  // Don't bring up the manager ROS API
  nodelet::V_string nargv;

  ros::NodeHandle nh_private("~");

  int queue_size;
  nh_private.param("queue_size", queue_size, 10);

  nodelet::M_string remap(ros::names::getRemappings());

  // STEREO UNDISTORT NODELET

  // leave in node namespace to get parameters
  std::string stereo_undistort_name = ros::this_node::getName();
  manager.load(stereo_undistort_name, "image_undistort/StereoUndistortNodelet",
               remap, nargv);
  ROS_INFO_STREAM("Started " << stereo_undistort_name << "/stereo_undistort"
                             << " nodelet.");

  // DEPTH NODELET
  std::string depth_name = ros::this_node::getName() + "/depth";

 // XmlRpc::XmlRpcValue depth_params;
  //ros::param::get(depth_name, depth_params);

  nodelet::M_string depth_remap;
  depth_remap["rect/first/image"] = ros::names::resolve("rect/first/image");
  depth_remap["rect/second/image"] = ros::names::resolve("rect/second/image");
  depth_remap["rect/first/camera_info"] =
      ros::names::resolve("rect/first/camera_info");
  depth_remap["rect/second/camera_info"] =
      ros::names::resolve("rect/second/camera_info");

  manager.load(depth_name, "image_undistort/DepthNodelet", depth_remap, nargv);
  ROS_INFO_STREAM("Started " << depth_name << " nodelet.");

  ros::spin();
  return 0;
}
