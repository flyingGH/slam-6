find_package(OpenCV 4.2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB viso
        "*.h"
        "*.cpp"
        )
add_executable(vo ${viso})
target_link_libraries(vo ${OpenCV_LIBS})