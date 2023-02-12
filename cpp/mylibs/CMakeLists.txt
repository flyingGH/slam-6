pybind11_add_module(mylibs mylibs.cpp)

# cf. https://stackoverflow.com/questions/6594796/how-do-i-make-cmake-output-into-a-bin-dir
set_target_properties(mylibs
        PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/pybind/lib")