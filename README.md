# Simple Ros node for undistorting images

Handles the ethz equidistant distortion model given by Kalibr
Other then that acts in a similar manner to image_proc with less functionality

Consists of two nodes

cam_info_reader_node: reads a Kalibr camchain yaml file and outputs a ros camera_info object
image_undistort_node: takes in a camera_info and image topic and outputs an undistorted image