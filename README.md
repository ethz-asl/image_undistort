#ROS node for undistorting images and performing stereo rectification

This repo contains two related ros nodes image_undistort_node and stereo_info_node

#image_undistort_node:
A simple node for undistorting images. Handles both plumb bob (aka radial-tangental) distortion and equidistant distortion models. It can either use standard ros camera_info topics or load camera models in a form that is compatiable with the camchain.yaml files produced by [Kalibr](https://github.com/ethz-asl/kalibr).

##The node has several possible use cases:

* **Undistort images.** The default usage of the node, outputing an undistorted version of an input image.
* **Modify the image resoultion and intrinsics.** The node supports projecting from and to any valid projection matrix and resolution.
* **Provide a camera_info topic for an image.** In this mode ros params and used to build a camera info message that is published in sync with the image messages. This allows the use of ros nodes that require camera info with devices and bags that do not provide it.

##Parameters:


