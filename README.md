#ROS node for undistorting images

A simple node for undistorting images. Handles both plumb bob (aka radial-tangental) distortion and equidistant distortion models. It can either use standard ros camera_info topics or load camera models in a form that is compatiable with the camchain.yaml files produced by [Kalibr](https://github.com/ethz-asl/kalibr).

By default the output is a undistorted image with the same projection matrix as the input camera, however it can create cameras with arbitrary projection matricies.
