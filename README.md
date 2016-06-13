#ROS node for undistorting images

Handles the equidistant distortion model and camchain.yaml files given by [Kalibr](https://github.com/ethz-asl/kalibr).

**Consists of two nodes**

- **cam_info_reader_node:** reads a Kalibr camchain yaml file and outputs a ros camera_info object
- **image_undistort_node:** takes in a camera_info and image topic and outputs an undistorted image
