cmake_minimum_required(VERSION 3.16)

# see here: https://github.com/PointCloudLibrary/pcl/issues/3680
# when this is fixed, we can remove the following 3 lines.
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(glog 0.5.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Ceres 2.1.0 REQUIRED)
find_package(g2o REQUIRED)
# for matplotlibcpp
find_package(PythonLibs REQUIRED)

add_definitions(${PCL_DEFINITIONS})

file(GLOB path_list *.cpp)
foreach(path ${path_list})
    get_filename_component(target_name ${path} NAME_WE)

    add_executable(${target_name} ${path})
    target_compile_options(${target_name} PUBLIC
            # 各種警告
            -Wall -Wextra -Wshadow -Wconversion -Wfloat-equal -Wno-char-subscripts
            # 数値関連エラー：オーバーフロー・未定義動作を検出
            -ftrapv -fno-sanitize-recover
            # デバッグ情報付与
            $<$<CONFIG:Debug>: -g>
            # 最適化
            $<$<CONFIG:Release>: -mtune=native -march=native -mfpmath=both -O2>
            )
    target_compile_features(${target_name} PUBLIC cxx_std_17)

    link_directories(${target_name} PUBLIC ${PCL_LIBRARY_DIRS})
    target_include_directories(${target_name} PUBLIC
            ${OpenCV_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
            ${G2O_INCLUDE_DIRS}
            # for matplotlibcpp
            ${PYTHON_INCLUDE_DIRS})
    target_link_libraries(${target_name} PUBLIC
            glog::glog
            Eigen3::Eigen
            Sophus::Sophus
            ${OpenCV_LIBS}
            ${PCL_LIBRARIES}
            Ceres::ceres
            g2o_core g2o_stuff
            # for matplotlibcpp
            ${PYTHON_LIBRARIES})
endforeach()