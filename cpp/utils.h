#pragma once

#include <boost/optional.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <chrono>
#include <iostream>
#include <string>

namespace util {
/**
 * @brief タイマークラス. ローカルスコープ内で使用して処理時間を計測.
 */
class Timer {
 public:
  Timer(const std::string& description) : _description(description) {
    _start_time = std::chrono::system_clock::now();
  }
  ~Timer() {
    const auto end = std::chrono::system_clock::now();
    const auto dur = end - _start_time;
    const auto msec =
        std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    std::cout << "[" << _description << "]: " << msec << " ms\n";
  }

 private:
  std::string _description;
  std::chrono::system_clock::time_point _start_time;
};

/**
 * @brief 設定ファイルクラス. Singleton でどこからでも即座に読み出し可能.
 */
class Config {
 public:
  static void read_ini(const std::string& path) { read_ini(path, _config_pt); }

  template <typename T>
  static T get_config(const std::string& key) {
    if (_config_pt.empty()) {
      throw std::runtime_error("config not read yet");
    }

    if (boost::optional<T> value = pt.get_optional<T>("Data.value")) {
      return value.get();
    } else {
      throw std::runtime_error("config reading error");
    }
  }

 private:
  Config() = default;
  ~Config() = default;
  ptree _config_pt;
}

}  // namespace util
