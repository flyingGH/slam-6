pybind11_add_module(mylibs mylibs.cpp)

target_compile_features(mylibs PUBLIC cxx_std_11)
target_compile_options(mylibs PUBLIC
        # 各種警告
        -Wall -Wextra -Wshadow -Wconversion -Wfloat-equal -Wno-char-subscripts
        # 数値関連エラー：オーバーフロー・未定義動作を検出
        -ftrapv -fno-sanitize-recover
        # デバッグ情報付与
        $<$<CONFIG:Debug>: -g>
        # 最適化
        $<$<CONFIG:Release>: -mtune=native -march=native -mfpmath=both -O2>)

# cf. https://stackoverflow.com/questions/6594796/how-do-i-make-cmake-output-into-a-bin-dir
set_target_properties(mylibs
        PROPERTIES
        PYTHON_EXECUTABLE /home/applejxd/.anyenv/envs/pyenv/versions/miniforge3/envs/py38pip/bin/python
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/pybind/lib")