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

  // DISPARITY NODELET
  std::string disparity_name = ros::this_node::getName() + "/disparity";

  XmlRpc::XmlRpcValue disparity_params;
  ros::param::get(disparity_name, disparity_params);
  if(!disparity_params.hasMember("approximate_sync")){
    ros::param::set(disparity_name + "/approximate_sync", true);
  }
  if(!disparity_params.hasMember("queue_size")){
    ros::param::set(disparity_name + "/queue_size", queue_size);
  }

  nodelet::M_string disparity_remap;
  disparity_remap["left/image_rect"] = ros::names::resolve("rect/left/image");
  disparity_remap["right/image_rect"] = ros::names::resolve("rect/right/image");
  disparity_remap["left/camera_info"] =
      ros::names::resolve("rect/left/camera_info");
  disparity_remap["right/camera_info"] =
      ros::names::resolve("rect/right/camera_info");

  manager.load(disparity_name, "stereo_image_proc/disparity", disparity_remap,
               nargv);
  ROS_INFO_STREAM("Started " << disparity_name << " nodelet.");

  // POINTCLOUD NODELET
  std::string pointcloud_name = ros::this_node::getName() + "/pointcloud";
  XmlRpc::XmlRpcValue pointcloud_params;

  ros::param::get(pointcloud_name, pointcloud_params);
  if(!pointcloud_params.hasMember("approximate_sync")){
    ros::param::set(pointcloud_name + "/approximate_sync", true);
  }
  if(!pointcloud_params.hasMember("queue_size")){
    ros::param::set(pointcloud_name + "/queue_size", queue_size);
  }

  nodelet::M_string pointcloud_remap;
  pointcloud_remap["left/camera_info"] =
      ros::names::resolve("rect/left/camera_info");
  pointcloud_remap["right/camera_info"] =
      ros::names::resolve("rect/right/camera_info");
  pointcloud_remap["left/image_rect_color"] =
      ros::names::resolve("rect/left/image");

  manager.load(pointcloud_name, "stereo_image_proc/point_cloud2",
               pointcloud_remap, nargv);
  ROS_INFO_STREAM("Started " << pointcloud_name << " nodelet.");

  ros::spin();
  return 0;
}
