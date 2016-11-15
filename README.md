#ROS node for undistorting images and performing stereo rectification

This repo contains two related ros nodes image_undistort_node and stereo_info_node

#image_undistort_node:
A simple node for undistorting images. Handles both plumb bob (aka radial-tangental) distortion and equidistant distortion models. It can either use standard ros camera_info topics or load camera models in a form that is compatiable with the camchain.yaml files produced by [Kalibr](https://github.com/ethz-asl/kalibr).

##The node has several possible use cases:

* **Undistort images.** The default usage of the node, outputing an undistorted version of an input image.
* **Modify the image resoultion and intrinsics.** The node supports projecting from and to any valid projection matrix and resolution.
* **Provide a camera_info topic for an image.** In this mode ros params and used to build a camera info message that is published in sync with the image messages. This allows the use of ros nodes that require camera info with devices and bags that do not provide it.

##Parameters:
* **queue size** The length of the queues the node uses for topics (default: 100).
* **input_camera_info_from_ros_params** If false the node will subscribe to a camera_info ros topic named input/camera_info to obtain the input camera parameters. If false the input camera parameters will be loaded from ros parameters. See the parameters format section for further details. (default: false).
* **output_camera_info_source** The source to use when obtaining the output camera parameters. The possible case-insensitive options are,
  * *"auto_generated"* The default value. In this mode "good" output parameters are automatically generated based on the input image. focal length is the average of fx and fy of the input, the center point is in the center of the image, R=I and translation is preserved. Resolution is set to the largest area that contains no empty pixels. The size of the output can also be modified with the *scale* parameter.
  * *"match_input"* The output projection matrix and resolution, exactly match the inputs.
  * *"ros_params"* The output camera parameters are loaded from ros parameters. See the parameters format section for further details.
  * *"camera_info"* The output parameters are found through subscribing to a camera_info ros topic named output/camera_info
* **input_camera_namespace** If the input camera parameters are loaded from ros parameters this is the namespace that will be searched. This is needed to allow both input and output to be loaded from parameters. (default: "input_camera")
* **output_camera_namespace** If the output camera parameters are loaded from ros parameters this is the namespace that will be searched. (default: "output_camera").
* **process_images** True to output a processed image, false if only a camera_info topic should be generated. (default: true).
* **undistort** True to undistort the images, false to keep the distortion. (default: true).
* **process_every_nth_frame** Used to temporarly downsample the images, if it is <= 1 every frame will be processed. (default: 1).
* **output_image_type** Converts the output image to the specified format, set to the empty string "" to preserve the input type. See [the cv_bridge tutorial](http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages) for possible format strings. (default: "").
* **scale** Only used if **output_camera_info_source** is set to "auto_generated". The output focal length will be multiplied by this value. This has the effect of resizing the image by this scale factor. (default: 1.0).
constexpr double kDefaultScale = 1.0;
* **publish_tf** True to publish the tf between the input and output image. If the undistortion involves changes to the rotation matrix the frame that the image is in will change. This tf gives that change. (default: true)
* **output_frame** The name of the frame of the output images. (default: "output_camera")
