#include <glog/logging.h>

#include <iostream>

int main(__attribute__((unused)) int argc, char* argv[]) {
  // 同一フォルダでの実行を想定
  FLAGS_log_dir = "../log";
  // glog の初期化
  google::InitGoogleLogging(argv[0]);
  // クラッシュ時にスタックトレースを吐くシグナルハンドラを設定
  google::InstallFailureSignalHandler();

  // ログ保存フォルダを指定(マクロは何故かうまく設定できない)
  google::SetLogDestination(google::INFO, "./logs/");
  google::SetLogDestination(google::WARNING, "./logs/");
  google::SetLogDestination(google::ERROR, "./logs/");

  uint32_t num_cookies = 42;
  LOG(INFO) << "Found " << num_cookies << " cookies";
}