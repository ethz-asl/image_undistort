image_undistort exists to handle all the odd situations image_proc doesn't quite cover. Some examples of this are
* working with images that don't have a camera_info topic
* undistortion of images using equidistant or other less common camera models
* turning a location in a distorted image into a bearing vector

If you have an image undistortion / stereo imaging problem that the library doesn't cover, create an issue and I'll look at adding it. Note that the automatic image size approach used will fail for cameras with a fov greater than 180 degrees.

This repo contains six related ros nodes-
* **[image_undistort_node](#image_undistort_node):** Undistorts and changes images intrinsics and resolution.
* **[stereo_info_node](#stereo_info_node):** Calculates the camera information needed for stereo rectification.
* **[stereo_undistort_node](#stereo_undistort_node):** Combines the functionality of the above two nodes to perform stereo image rectification.
* **[depth_node](#depth_node):** Converts two undistorted images and their camera information into a disparity image and a pointcloud.
* **[dense_stereo_node](#dense_stereo_node):** Performs the full dense stereo estimation (internally this node is just the stereo_undistort nodelet and the depth nodelet).
* **[point_to_bearing_node](#point_to_bearing_node):** Takes in a 2D image location and transforms it into a bearing vector.

## Dependencies
Image undistort depends on ROS, OpenCV and Eigen. The point to bearing node also depends on NLopt (installed with `apt install libnlopt-dev`) and will only be built if it is found. 

## Supported Camera and Distortion Models
The only supported output is the pinhole camera model with no distortion. 
Supported input models:

* Pinhole with no distortion
* Pinhole with radial-tangential distortion
* Pinhole with equidistant distortion
* Omnidirectional with no distortion
* Omindirectional with rad-tan distortion
* FOV
* Unified
* Extended Unified
* Double Sphere 


# image_undistort_node:
A simple node for undistorting images. Handles plumb bob (aka radial-tangential), fov and equidistant distortion models. It can either use standard ros camera_info topics or load camera models in a form that is compatible with the camchain.yaml files produced by [Kalibr](https://github.com/ethz-asl/kalibr). Note this node can also be run as a nodelet named `image_undistort/ImageUndistortNodelet`.

## The node has several possible use cases:

* **Undistort images.** The default usage of the node, outputting an undistorted version of an input image.
* **Modify the image resolution and intrinsics.** The node supports projecting from and to any valid projection matrix and resolution.
* **Provide a camera_info topic for an image.** In this mode ros params are used to build a camera info message that is published in sync with the image messages. This allows the use of ros nodes that require camera info with devices and bags that do not provide it.

## Parameters:
* **queue size** The length of the queues the node uses for topics (default: 10).
* **input_camera_info_from_ros_params** If false the node will subscribe to a camera_info ros topic named input/camera_info to obtain the input camera parameters. If true the input camera parameters will be loaded from ros parameters. See the parameters format section for further details. (default: false).
* **output_camera_info_source** The source to use when obtaining the output camera parameters. The possible case-insensitive options are,
  * *"auto_generated"* The default value. In this mode "good" output parameters are automatically generated based on the input image. focal length is the average of fx and fy of the input, the center point is in the center of the image, R=I and translation is preserved. Resolution is set to the largest area that contains no empty pixels. The size of the output can also be modified with the *scale* parameter.
  * *"match_input"* The output projection matrix and resolution, exactly match the inputs.
  * *"ros_params"* The output camera parameters are loaded from ros parameters. See the parameters format section for further details.
  * *"camera_info"* The output parameters are found through subscribing to a camera_info ros topic named output/camera_info
* **input_camera_namespace** If the input camera parameters are loaded from ros parameters this is the namespace that will be searched. This is needed to allow both input and output to be loaded from parameters. (default: "input_camera")
* **output_camera_namespace** If the output camera parameters are loaded from ros parameters this is the namespace that will be searched. (default: "output_camera").
* **process_image** True to output a processed image, false if only a camera_info topic should be generated. (default: true).
* **undistort_image** True to undistort the images, false to keep the distortion. (default: true).
* **process_every_nth_frame** Used to temporarily down-sample the images, if it is <= 1 every frame will be processed. (default: 1).
* **output_image_type** Converts the output image to the specified format, set to the empty string "" to preserve the input type. See [the cv_bridge tutorial](http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages) for possible format strings. (default: "").
* **scale** Only used if **output_camera_info_source** is set to "auto_generated" or "match_input". The output focal length will be multiplied by this value. If "auto_generated" is set the image size will also be increased by this factor. (default: 1.0).
* **publish_tf** True to publish the tf between the input and output image. If the undistortion involves changes to the rotation matrix the frame that the image is in will change. This tf gives that change. (default: true)
* **output_frame** The name of the frame of the output images. (default: "output_camera")
* **rename_input_frame** If the input frame should be renamed in the published topics and tf tree. (default: false)
* **input_frame** Only used if **rename_input_frame** is true. The name of the frame of the input images. (default: "input_camera")
* **rename_radtan_plumb_bob** If true the radial-tangential distortion model will be called "plumb_bob" in the output camera_info, this is needed by some ros image processing nodes. If false it will be called "radtan". (default: false).

## Input/Output Topics
Many of these topics are dependent on the parameters set above and may not appear or may be renamed under some settings.
* **input/image** input image topic
* **input/camera_info** input camera info topic
* **output/image** output image topic
* **output/camera_info** output camera info topic

## Loading Camera Information from ROS Parameters:

Camera information can be loaded from ROS parameters. These parameters are typically set using <rosparam file="path_to_yaml_file"/>. The format used by this node is compatible with the camchains generated by [Kalibr](https://github.com/ethz-asl/kalibr). The follow steps are used when loading this information.

1. A 3x3 intrinscs matrix named **K** is searched for. If it is found it is loaded. If it is not found a 1x4 vector named **intrinsics** is loaded, this contains the parameters (fx, fy, cx, cy). If neither parameters are given the node displays an error and terminates.
2. A 1x2 vector named **resolution** is loaded which contains the parameters (width, height). Again, if not given the node displays an error and terminates.
3. A 4x4 transformation matrix **T_cn_cnm1** is searched for. If it is found it is loaded. Otherwise it is searched for under the name **T** and if found loaded. If neither are found the node continues.
4. A 4x3 projection matrix **P** is searched for. If it is found it is loaded. If **P** was found but **T** was not, **P** and **K** are used to construct **T**, otherwise **T** is set to identity. If **P** was not found it is constructed from **K** and **T**.
5. If an output is being loaded, the loading of parameters is completed. For input cameras the distortion properties are now loaded
6. A 1xn vector **D** is loaded. If it is not found or is less than 5 elements long it is padded with zeros.
7. A string **distortion_model** is loaded and converted to lower-case. If it is not found it is set to "radtan".

# stereo_info_node:
A node that takes in the properties of two cameras and outputs the camera info required to rectify them so that stereo reconstruction can be performed. The rectification is performed such that only x translation is present between the cameras. The focal points are in the image centers, fx=fy and the image resolution is set to be the largest frame that contains no empty pixels. Note this node can also be run as a nodelet named `image_undistort/StereoInfoNodelet`.

## Parameters:
* **queue size** The length of the queues the node uses for topics (default: 10).
* **input_camera_info_from_ros_params** If false the node will subscribe to a camera_info ros topic named input/camera_info to obtain the input camera parameters. If true the input camera parameters will be loaded from ros parameters. See the parameters format section for further details. (default: false).
* **first_camera_namespace** If the first camera parameters are loaded from ros parameters this is the namespace that will be searched. (default: "first_camera")
* **second_camera_namespace** If the second camera parameters are loaded from ros parameters this is the namespace that will be searched. (default: "second_camera").
* **scale** Only used if **output_camera_info_source** is set to "auto_generated". The output focal length will be multiplied by this value. This has the effect of resizing the image by this scale factor. (default: 1.0).
**rename_radtan_plumb_bob** If true the radial-tangential distortion model will be called "plumb_bob" in the output camera_info, this is needed by some ros image processing nodes. If false it will be called "radtan". (default: false).

## Input/Output Topics
Many of these topics are dependent on the parameters set above and may not appear or may be renamed under some settings.
* **raw/first/image** first input image topic, only needed if loading camera parameters from ros params (used for timing information) 
* **raw/second/image** second input image topic, only needed if loading camera parameters from ros params (used for timing information) 
* **raw/first/camera_info** first input camera info topic
* **raw/second/camera_info** second input camera info topic
* **rect/first/camera_info** first output camera info topic
* **rect/second/camera_info** second output camera info topic

# stereo_undistort_node:
A node that takes in the images and properties of two cameras and outputs rectified stereo images with their corresponding camera parameters. The rectification is performed such that only x translation is present between the cameras. The focal points are in the image centers, fx=fy and the image resolution is set to be the largest frame that contains no empty pixels. Note this node can also be run as a nodelet named `image_undistort/StereoUndistortNodelet`.

## Parameters:
* **queue size** The length of the queues the node uses for topics (default: 10).
* **input_camera_info_from_ros_params** If false the node will subscribe to a camera_info ros topic named input/camera_info to obtain the input camera parameters. If true the input camera parameters will be loaded from ros parameters. See the parameters format section for further details. (default: false).
* **first_camera_namespace** If the first camera parameters are loaded from ros parameters this is the namespace that will be searched. (default: "first_camera")
* **second_camera_namespace** If the second camera parameters are loaded from ros parameters this is the namespace that will be searched. (default: "second_camera").
* **scale** Only used if **output_camera_info_source** is set to "auto_generated". The output focal length will be multiplied by this value. This has the effect of resizing the image by this scale factor. (default: 1.0).
* **process_every_nth_frame** Used to temporarily down-sample the images, if it is <= 1 every frame will be processed. (default: 1).
* **output_image_type** Converts the output images to the specified format, set to the empty string "" to preserve the input type. See [the cv_bridge tutorial](http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages) for possible format strings. (default: "").
* **scale** The output focal length will be multiplied by this value. This has the effect of resizing the image by this scale factor. (default: 1.0).
* **T_invert** Only used if loading parameters from ros params. True to invert the given transformations. (default: false)
* **publish_tf** True to publish the tf between the first input and output image. If the undistortion involves changes to the rotation matrix the frame that the image is in will change. This tf gives that change. (default: true)
* **first_output_frame** The name of the frame of the first camera output images. (default: "first_camera_rect")
* **second_output_frame** The name of the frame of the second camera output images. (default: "second_camera_rect")
* **rename_input_frame** If the input frame should be renamed in the published topics and tf tree. (default: false)
* **first_input_frame** Only used if **rename_input_frame** is true. The name of the frame of the first input images. (default: "first_camera")
* **second_input_frame** Only used if **rename_input_frame** is true. The name of the frame of the second input images. (default: "second_camera")
* **rename_radtan_plumb_bob** If true the radial-tangential distortion model will be called "plumb_bob" in the output camera_info, this is needed by some ros image processing nodes. If false it will be called "radtan". (default: false).

## Input/Output Topics
Many of these topics are dependent on the parameters set above and may not appear or may be renamed under some settings.
* **raw/first/image** first input image topic
* **raw/second/image** second input image topic
* **raw/first/camera_info** first input camera info topic
* **raw/second/camera_info** second input camera info topic
* **rect/first/image** first output image topic
* **rect/second/image** second output image topic
* **rect/first/camera_info** first output camera info topic
* **rect/second/camera_info** second output camera info topic

# depth_node:
A node that takes in the rectified images and properties of two cameras and outputs a disparity image and a pointcloud. The node uses the camera_info topics to figure out which camera is the left one and which is the right one. Internally the node makes use of the opencv stereo block matcher to perform the depth estimation. Note this node can also be run as a nodelet named `image_undistort/DepthNodelet`.

## Parameters:
* **queue size** The length of the queues the node uses for topics. (default: 10)
* **pre_filter_type** The prefilter type (possible values: 'xsobel', 'normalized_response', default: 'xsobel')
* **pre_filter_size** The size of the prefilter used in StereoBM. (default: 9)
* **pre_filter_cap** The upper cap on the prefilter used in StereoBM. (default: 31)
* **sad_window_size** The window size used when performing the stereo matching, note the efficiency of the implementation reduces if this value is greater than 21 (default: 21)
* **min_disparity** The minimum disparity checked in StereoBM. (default: 0)
* **num_disparities** The number of disparities checked in StereoBM. (default: 64)
* **texture_threshold** Minimum texture a patch requires to be matched in StereoBM. (default: 10)
* **uniqueness_ratio** Minimum margin by which the best matching disparity must 'win' in StereoBM. (default: 15)
* **speckle_range** Parameter used for removing speckle in StereoBM. (default: 0)
* **speckle_window_size** Window size used for speckle removal in StereoBM. (default: 0)
* **use_sgbm** Use SGBM (Semi-Global Block Matching) instead of BM (Block Matching)? (default: false)
* **p1** The first parameter controlling the disparity smoothness, only available in SGBM (default: 120)
* **p2** The second parameter controlling the disparity smoothness, only available in SGBM (default: 240)
* **disp_12_max_diff** Maximum allowed difference (in integer pixel units) in the left-right disparity check, only available in SGBM (default: -1)
* **use_mode_HH** Run the full-scale two-pass dynamic programming algorithm. It will consume O(W*H*numDisparities) bytes, which is large for 640x480 stereo and huge for HD-size pictures. Only available in SGBM (default: false)
* **do_median_blur** Apply median blur to the final disparity image (default: true)

## Input/Output Topics
* **rect/first/image** first input image topic
* **rect/second/image** second input image topic
* **rect/first/camera_info** first input camera info topic
* **rect/second/camera_info** second input camera info topic
* **disparity/image** output disparity image
* **pointcloud** output pointcloud
* **freespace_pointcloud** output freespace pointcloud

# dense_stereo_node:
A node for producing dense stereo images. Internally this node simply combines 2 nodelets.
* **image_undistort/StereoUndistortNodelet** Used to set up the stereo system and rectify the images.
* **image_undistort/DepthNodelet** Generates disparity images and pointclouds from the rectified images.

## Parameters:
* **queue size** The length of the queues the node uses for topics (default: 10).
* **input_camera_info_from_ros_params** If false the node will subscribe to a camera_info ros topic named input/camera_info to obtain the input camera parameters. If true the input camera parameters will be loaded from ros parameters. See the parameters format section for further details. (default: false).
* **first_camera_namespace** If the first camera parameters are loaded from ros parameters this is the namespace that will be searched. (default: "first_camera")
* **second_camera_namespace** If the second camera parameters are loaded from ros parameters this is the namespace that will be searched. (default: "second_camera").
* **scale** Only used if **output_camera_info_source** is set to "auto_generated". The output focal length will be multiplied by this value. This has the effect of resizing the image by this scale factor. (default: 1.0).
* **T_invert** Only used if loading parameters from ros params. True to invert the given transformations. (default: false)
* **process_every_nth_frame** Used to temporarily down-sample the images, if it is <= 1 every frame will be processed. (default: 1).
* **output_image_type** Converts the output images to the specified format, set to the empty string "" to preserve the input type. See [the cv_bridge tutorial](http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages) for possible format strings. (default: "").
* **scale** The output focal length will be multiplied by this value. This has the effect of resizing the image by this scale factor. (default: 1.0).
* **publish_tf** True to publish the tf between the first input and output image. If the undistortion involves changes to the transformation matrix the frame that the image is in will change, this occurs during most image rectifications. This tf gives that change. (default: true)
* **output_frame** The name of the frame of the output images. (default: "first_camera_rect")
* **rename_input_frame** If the input frame should be renamed in the published topics and tf tree. (default: false)
* **first_input_frame** Only used if **rename_input_frame** is true. The name of the frame of the first input images. (default: "first_camera")
* **second_input_frame** Only used if **rename_input_frame** is true. The name of the frame of the second input images. (default: "second_camera")
**rename_radtan_plumb_bob** If true the radial-tangential distortion model will be called "plumb_bob" in the output camera_info, this is needed by some ros image processing nodes. If false it will be called "radtan". (default: false).
* **pre_filter_type** The prefilter type (possible values: 'xsobel', 'normalized_response', default: 'xsobel')
* **pre_filter_size** The size of the prefilter used in StereoBM. (default: 9)
* **pre_filter_cap** The upper cap on the prefilter used in StereoBM. (default: 31)
* **sad_window_size** The window size used when performing the stereo matching, note the efficiency of the implementation reduces if this value is greater than 21 (default: 21)
* **min_disparity** The minimum disparity checked in StereoBM. (default: 0)
* **num_disparities** The number of disparities checked in StereoBM. (default: 64)
* **texture_threshold** Minimum texture a patch requires to be matched in StereoBM. (default: 10)
* **uniqueness_ratio** Minimum margin by which the best matching disparity must 'win' in StereoBM. (default: 15)
* **speckle_range** Parameter used for removing speckle in StereoBM. (default: 0)
* **speckle_window_size** Window size used for speckle removal in StereoBM. (default: 0)
* **use_sgbm** Use SGBM (Semi-Global Block Matching) instead of BM (Block Matching)? (default: false)
* **p1** The first parameter controlling the disparity smoothness, only available in SGBM (default: 120)
* **p2** The second parameter controlling the disparity smoothness, only available in SGBM (default: 240)
* **disp_12_max_diff** Maximum allowed difference (in integer pixel units) in the left-right disparity check, only available in SGBM (default: -1)
* **use_mode_HH** Run the full-scale two-pass dynamic programming algorithm. It will consume O(W*H*numDisparities) bytes, which is large for 640x480 stereo and huge for HD-size pictures. Only available in SGBM (default: false)
* **do_median_blur** Apply median blur to the final disparity image (default: true)

## Input/Output Topics
Many of these topics are dependent on the parameters set above and may not appear or may be renamed under some settings.
* **raw/first/image** first input image topic
* **raw/second/image** second input image topic
* **raw/first/camera_info** first input camera info topic
* **raw/second/camera_info** second input camera info topic
* **rect/first/image** first output rectified image topic
* **rect/second/image** second output rectified image topic
* **rect/first/camera_info** first output camera info topic
* **rect/second/camera_info** second output camera info topic
* **disparity** output disparity image topic
* **pointcloud** output pointcloud topic

# point_to_bearing_node:
A node for converting a point in a distorted image to a unit bearing vector. Note this node can also be run as a nodelet named `image_undistort/PointToBearingNodelet`.

## Parameters:
* **queue size** The length of the queues the node uses for topics (default: 10).
* **input_camera_info_from_ros_params** If false the node will subscribe to a camera_info ros topic named input/camera_info to obtain the input camera parameters. If true the input camera parameters will be loaded from ros parameters. See the parameters format section for further details. (default: false).

## Input/Output Topics
Many of these topics are dependent on the parameters set above and may not appear or may be renamed under some settings.
* **input/camera_info** camera info topic
* **image_point** input location of the point of interest in the image
* **bearing** unit bearing to point
