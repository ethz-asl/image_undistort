#ROS node for undistorting images

A simple node for undistorting images. Handles both plumb bob (aka radial-tangental) distortion and equidistant distortion models. It can either use standard ros camera_info topics or load camera models in a form that is compatiable with the camchain.yaml files produced by [Kalibr](https://github.com/ethz-asl/kalibr).

The library has several possible use cases:

Undistort images. The default usage of the node, subscribing to an image topic and outputing an undistorted version with the same projection matrix. The default behaviour is to read the original camera parameters from a ros topic "cam_info", however by setting "output_camera_info_from_yaml" to true the parameters can be loaded from a yaml file. The format of this yaml file is compatible with that produced by [Kalibr](https://github.com/ethz-asl/kalibr).

Modify the image resoultion and intrinsics. If "output_camera_info_from_yaml" is set to true the output resolution, camera matrix (K), rotation matrix (R) and projection matrix (P) may be specified. The output camera parameters will be set to conform to the projection matrix values. If no projection matrix is set it will be calculated via R*K and if no rotation matrix is set the identity transform will be used. If neither the projection matrix or camera matrix is given the node will print an error and exit. If the "undistort" parameter is true (the default behaviour) the distortion will also be removed.

Provide a cam_info topic for a ros image topic. In this mode a yaml file is loaded and used to build a camera info message that is published in sync with the image messages. This allows the use of ros nodes that require camera info with devices and bags that do not provide it.
