#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

int main() {
    std::cout << "Eigen Version: "
    << EIGEN_WORLD_VERSION << "."
    << EIGEN_MAJOR_VERSION << "."
    << EIGEN_MINOR_VERSION << std::endl;

    std::cout << "OpenCV Version: "
    << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << std::endl;
}
