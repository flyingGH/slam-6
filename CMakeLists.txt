cmake_minimum_required(VERSION 3.21)
project(slam-practice)

find_package(pybind11 REQUIRED)

add_subdirectory(cpp/mono-vo)
add_subdirectory(cpp/samples)
add_subdirectory(pybind/src)
