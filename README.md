This repo contains four related ros nodes-
* **[image_undistort_node](https://github.com/ethz-asl/image_undistort#image_undistort_node):** Undistorts and changes images intrinsics and resolution.
* **[stereo_info_node](https://github.com/ethz-asl/image_undistort#stereo_info_node):** Calculates the camera information needed for stereo rectification.
* **[stereo_undistort_node](https://github.com/ethz-asl/image_undistort#stereo_undistort_node):** Combines the functionality of the above two nodes to perform stereo image rectification.
* **[dense_stereo_node](https://github.com/ethz-asl/image_undistort#dense_stereo_node):** Performs dense stereo estimation.

#image_undistort_node:
A simple node for undistorting images. Handles plumb bob (aka radial-tangential), fov and equidistant distortion models. It can either use standard ros camera_info topics or load camera models in a form that is compatible with the camchain.yaml files produced by [Kalibr](https://github.com/ethz-asl/kalibr). Note this node can also be run as a nodelet named image_undistort/ImageUndistort

##The node has several possible use cases:

* **Undistort images.** The default usage of the node, outputting an undistorted version of an input image.
* **Modify the image resolution and intrinsics.** The node supports projecting from and to any valid projection matrix and resolution.
* **Provide a camera_info topic for an image.** In this mode ros params are used to build a camera info message that is published in sync with the image messages. This allows the use of ros nodes that require camera info with devices and bags that do not provide it.

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
* **process_every_nth_frame** Used to temporarily down-sample the images, if it is <= 1 every frame will be processed. (default: 1).
* **output_image_type** Converts the output image to the specified format, set to the empty string "" to preserve the input type. See [the cv_bridge tutorial](http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages) for possible format strings. (default: "").
* **scale** Only used if **output_camera_info_source** is set to "auto_generated". The output focal length will be multiplied by this value. This has the effect of resizing the image by this scale factor. (default: 1.0).
* **publish_tf** True to publish the tf between the input and output image. If the undistortion involves changes to the rotation matrix the frame that the image is in will change. This tf gives that change. (default: true)
* **output_frame** The name of the frame of the output images. (default: "output_camera")
* **rename_input_frame** If the input frame should be renamed in the published topics and tf tree. (default: false)
* **input_frame** Only used if **rename_input_frame** is true. The name of the frame of the input images. (default: "input_camera")
* **rename_radtan_plumb_bob** If true the radial-tangential distortion model will be called "plumb_bob" in the output camera_info, this is needed by some ros image processing nodes. If false it will be called "radtan". (default: false).

##Input/Output Topics
Many of these topics are dependent on the parameters set above and may not appear or may be renamed under some settings.
* **input/image** input image topic
* **input/camera_info** input camera info topic
* **output/image** output image topic
* **output/camera_info** output camera info topic

##Loading Camera Information from ROS Parameters:

Camera information can be loaded from ROS parameters. These parameters are typically set using <rosparam file="path_to_yaml_file"/>. The format used by this node is compatible with the camchains generated by [Kalibr](https://github.com/ethz-asl/kalibr). The follow steps are used when loading this information.

1. A 3x3 intrinscs matrix named **K** is searched for. If it is found it is loaded. If it is not found a 1x4 vector named **intrinsics** is loaded, this contains the parameters (fx, fy, cx, cy). If neither parameters are given the node displays an error and terminates.
2. A 1x2 vector named **resolution** is loaded which contains the parameters (width, height). Again, if not given the node displays an error and terminates.
3. A 4x4 transformation matrix **T** is searched for. If it is found it is loaded. Otherwise it is searched for under the name **T_cn_cnm1** and if found loaded. If neither are found the node continues.
4. A 4x3 projection matrix **P** is searched for. If it is found it is loaded. If **P** was found but **T** was not, **P** and **K** are used to construct **T**, otherwise **T** is set to identity. If **P** was not found it is constructed from **K** and **T**.
5. If an output is being loaded, the loading of parameters is completed. For input cameras the distortion properties are now loaded
6. A 1xn vector **D** is loaded. If it is not found or is less than 5 elements long it is padded with zeros.
7. A string **distortion_model** is loaded and converted to lower-case. If it is not found it is set to "radtan".

#stereo_info_node:
A node that takes in the properties of two cameras and outputs the camera info required to rectify them so that stereo reconstruction can be performed. The rectification is performed such that only x translation is present between the cameras. The focal points are in the image centers, fx=fy and the image resolution is set to be the largest frame that contains no empty pixels. Note this node can also be run as a nodelet named image_undistort/StereoInfo

##Parameters:
* **queue size** The length of the queues the node uses for topics (default: 100).
* **input_camera_info_from_ros_params** If false the node will subscribe to a camera_info ros topic named input/camera_info to obtain the input camera parameters. If false the input camera parameters will be loaded from ros parameters. See the parameters format section for further details. (default: false).
* **left_camera_namespace** If the left camera parameters are loaded from ros parameters this is the namespace that will be searched. (default: "left_camera")
* **right_camera_namespace** If the right camera parameters are loaded from ros parameters this is the namespace that will be searched. (default: "right_camera").
* **scale** Only used if **output_camera_info_source** is set to "auto_generated". The output focal length will be multiplied by this value. This has the effect of resizing the image by this scale factor. (default: 1.0).
**rename_radtan_plumb_bob** If true the radial-tangential distortion model will be called "plumb_bob" in the output camera_info, this is needed by some ros image processing nodes. If false it will be called "radtan". (default: false).

##Input/Output Topics
Many of these topics are dependent on the parameters set above and may not appear or may be renamed under some settings.
* **raw/left/image** left input image topic, only needed if loading camera parameters from ros params (used for timing information) 
* **raw/right/image** right input image topic, only needed if loading camera parameters from ros params (used for timing information) 
* **raw/left/camera_info** left input camera info topic
* **raw/right/camera_info** right input camera info topic
* **rect/left/camera_info** left output camera info topic
* **rect/right/camera_info** right output camera info topic

#stereo_undistort_node:
A node that takes in the images and properties of two cameras and outputs rectified stereo images with their corresponding camera parameters. The rectification is performed such that only x translation is present between the cameras. The focal points are in the image centers, fx=fy and the image resolution is set to be the largest frame that contains no empty pixels. Note this node can also be run as a nodelet named image_undistort/StereoUndistort

##Parameters:
* **queue size** The length of the queues the node uses for topics (default: 100).
* **input_camera_info_from_ros_params** If false the node will subscribe to a camera_info ros topic named input/camera_info to obtain the input camera parameters. If false the input camera parameters will be loaded from ros parameters. See the parameters format section for further details. (default: false).
* **left_camera_namespace** If the left camera parameters are loaded from ros parameters this is the namespace that will be searched. (default: "left_camera")
* **right_camera_namespace** If the right camera parameters are loaded from ros parameters this is the namespace that will be searched. (default: "right_camera").
* **scale** Only used if **output_camera_info_source** is set to "auto_generated". The output focal length will be multiplied by this value. This has the effect of resizing the image by this scale factor. (default: 1.0).
* **process_every_nth_frame** Used to temporarily down-sample the images, if it is <= 1 every frame will be processed. (default: 1).
* **output_image_type** Converts the output images to the specified format, set to the empty string "" to preserve the input type. See [the cv_bridge tutorial](http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages) for possible format strings. (default: "").
* **scale** The output focal length will be multiplied by this value. This has the effect of resizing the image by this scale factor. (default: 1.0).
* **publish_tf** True to publish the tf between the left input and output image. If the undistortion involves changes to the rotation matrix the frame that the image is in will change. This tf gives that change. (default: true)
* **output_frame** The name of the frame of the output images. (default: "left_camera_rect")
* **rename_input_frame** If the input frame should be renamed in the published topics and tf tree. (default: false)
* **left_input_frame** Only used if **rename_input_frame** is true. The name of the frame of the left input images. (default: "left_camera")
* **right_input_frame** Only used if **rename_input_frame** is true. The name of the frame of the right input images. (default: "right_camera")
**rename_radtan_plumb_bob** If true the radial-tangential distortion model will be called "plumb_bob" in the output camera_info, this is needed by some ros image processing nodes. If false it will be called "radtan". (default: false).

##Input/Output Topics
Many of these topics are dependent on the parameters set above and may not appear or may be renamed under some settings.
* **raw/left/image** left input image topic
* **raw/right/image** right input image topic
* **raw/left/camera_info** left input camera info topic
* **raw/right/camera_info** right input camera info topic
* **rect/left/image** left output image topic
* **rect/right/image** right output image topic
* **rect/left/camera_info** left output camera info topic
* **rect/right/camera_info** right output camera info topic

#dense_stereo_node:
A node for producing dense stereo images. Internally this node simply combines 3 nodelets.
* **image_undistort/StereoUndistort** Used to set up the stereo system and rectify the images.
* **stereo_image_proc/disparity** Standard ros nodelet for generating a disparity image from a rectified stereo pair.
* **stereo_image_proc/point_cloud2** Standard ros nodelet for generating a color pointcloud from an image and disparity image pair.

##Parameters:
* **queue size** The length of the queues the node uses for topics (default: 10).
* **input_camera_info_from_ros_params** If false the node will subscribe to a camera_info ros topic named input/camera_info to obtain the input camera parameters. If false the input camera parameters will be loaded from ros parameters. See the parameters format section for further details. (default: false).
* **left_camera_namespace** If the left camera parameters are loaded from ros parameters this is the namespace that will be searched. (default: "left_camera")
* **right_camera_namespace** If the right camera parameters are loaded from ros parameters this is the namespace that will be searched. (default: "right_camera").
* **scale** Only used if **output_camera_info_source** is set to "auto_generated". The output focal length will be multiplied by this value. This has the effect of resizing the image by this scale factor. (default: 1.0).
* **process_every_nth_frame** Used to temporarily down-sample the images, if it is <= 1 every frame will be processed. (default: 1).
* **output_image_type** Converts the output images to the specified format, set to the empty string "" to preserve the input type. See [the cv_bridge tutorial](http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages) for possible format strings. (default: "").
* **scale** The output focal length will be multiplied by this value. This has the effect of resizing the image by this scale factor. (default: 1.0).
* **publish_tf** True to publish the tf between the left input and output image. If the undistortion involves changes to the transformation matrix the frame that the image is in will change, this occurs during most image rectifications. This tf gives that change. (default: true)
* **output_frame** The name of the frame of the output images. (default: "left_camera_rect")
* **rename_input_frame** If the input frame should be renamed in the published topics and tf tree. (default: false)
* **left_input_frame** Only used if **rename_input_frame** is true. The name of the frame of the left input images. (default: "left_camera")
* **right_input_frame** Only used if **rename_input_frame** is true. The name of the frame of the right input images. (default: "right_camera")
**rename_radtan_plumb_bob** If true the radial-tangential distortion model will be called "plumb_bob" in the output camera_info, this is needed by some ros image processing nodes. If false it will be called "radtan". (default: false).

###Note:
All of the standard parameters of **stereo_image_proc/disparity** and **stereo_image_proc/point_cloud2** are also accessible and settable. These parameters appear in the **~/disparity** and **~/point_cloud2** namespaces respectively. Also note that unlike the standard behavior, this node sets approximate_sync to true by default.  See [the stereo_image_proc page](http://wiki.ros.org/stereo_image_proc) for up to date parameters and default values.


##Input/Output Topics
Many of these topics are dependent on the parameters set above and may not appear or may be renamed under some settings.
* **raw/left/image** left input image topic
* **raw/right/image** right input image topic
* **raw/left/camera_info** left input camera info topic
* **raw/right/camera_info** right input camera info topic
* **rect/left/image** left output rectified image topic
* **rect/right/image** right output rectified image topic
* **rect/left/camera_info** left output camera info topic
* **rect/right/camera_info** right output camera info topic
* **disparity** output disparity image topic
* **points2** output pointcloud topic
* **dense_stereo_disparity/set_parameters** service for setting the stereo image generation properties. See [stereo_image_proc](http://wiki.ros.org/stereo_image_proc) for details.
