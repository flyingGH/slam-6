find_package(OpenCV 4.2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB viso
        "*.h"
        "*.cpp"
        )

add_executable(vo ${viso})
target_compile_features(vo PUBLIC cxx_std_11)
target_compile_options(vo PUBLIC
        # 各種警告
        -Wall -Wextra -Wshadow -Wconversion -Wfloat-equal -Wno-char-subscripts
        # 数値関連エラー：オーバーフロー・未定義動作を検出
        -ftrapv -fno-sanitize-recover
        # デバッグ情報付与
        $<$<CONFIG:Debug>: -g>
        # 最適化
        $<$<CONFIG:Release>: -mtune=native -march=native -mfpmath=both -O2>)
target_link_libraries(vo ${OpenCV_LIBS})