cmake_minimum_required(VERSION 3.21)
project(slam-practice)

set(CMAKE_CXX_STANDARD 11)

find_package(pybind11 REQUIRED)
add_subdirectory(cpp/mylibs)
add_subdirectory(cpp/mono-vo)
add_subdirectory(cpp/samples)
