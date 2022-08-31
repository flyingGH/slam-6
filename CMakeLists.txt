cmake_minimum_required(VERSION 3.21)
project(pybind11_practice)

set(CMAKE_CXX_STANDARD 11)

find_package(pybind11 REQUIRED)
add_subdirectory(src/mylibs)
add_subdirectory(src/mono-vo)

