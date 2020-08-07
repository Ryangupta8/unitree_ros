#ifndef ORIENTATION_UTILS_H
#define ORIENTATION_UTILS_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace math_utils{
  void convert(const Eigen::Quaterniond & from, Eigen::AngleAxisd & to);
  void convert(const Eigen::AngleAxisd & from, Eigen::Vector3d & to);
  void convert(const Eigen::Quaterniond & from, Eigen::Vector3d & to);

  void compute_quat_error(const Eigen::Quaterniond & des,
    					  const Eigen::Quaterniond & current,
    					  Eigen::Vector3d & error);

  void printQuat(const Eigen::Quaterniond & quat);

}

#endif
