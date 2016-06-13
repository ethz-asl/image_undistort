#ifndef UNDISTORTER_H
#define UNDISTORTER_H

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <iostream>

#include <cv.h>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"

class Undistorter {
 public:
  Undistorter(const Eigen::Matrix3d& K, const std::vector<double>& D,
              const cv::Size& resolution, const bool using_radtan,
              const double zoom = 1.0);

  void undistortImage(const cv::Mat& image, cv::Mat* undistored_image);

 private:
  void distortPixel(const Eigen::Matrix3d& K, const std::vector<double>& D,
                    const double zoom, const double u, const double v,
                    double* ud, double* vd);

  cv::Mat map_x_;
  cv::Mat map_y_;

  bool using_radtan_;
};

#endif