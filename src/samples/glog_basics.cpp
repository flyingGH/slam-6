#include <iostream>
#include <glog/logging.h>

int main(__attribute__((unused)) int argc, char* argv[]) {
    // 同一フォルダでの実行を想定
    FLAGS_log_dir = "../log";
    // glog の初期化
    google::InitGoogleLogging(argv[0]);
    // クラッシュ時にスタックトレースを吐くうシグナルハンドラを設定
    google::InstallFailureSignalHandler();

    uint32_t num_cookies = 42;
    LOG(INFO) << "Found " << num_cookies << " cookies";
}