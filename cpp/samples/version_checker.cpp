#include <ceres/ceres.h>

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
  // https://gitlab.com/libeigen/eigen/-/blob/master/Eigen/src/Core/util/Macros.h
  std::cout << "Eigen Version: " << EIGEN_WORLD_VERSION << "."
            << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;
  // https://github.com/opencv/opencv/blob/9208dcb07c015e1fda44e40bb07b43c700b4bf46/modules/core/include/opencv2/core/version.hpp
  std::cout << "OpenCV Version: " << CV_VERSION << std::endl;
  // https://github.com/ceres-solver/ceres-solver/blob/1274743609bc59621adc9e311cdeeaad7eb65201/include/ceres/version.h
  std::cout << "Ceres-Solver Version: " << CERES_VERSION_STRING << std::endl;
}
