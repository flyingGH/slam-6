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
  static void read_ini(const std::string& path) {
    boost::property_tree::read_ini(path, Config::_pt);
  }

  template <typename T>
  static T get_config(const std::string& key) {
    if (Config::_pt.empty()) {
      throw std::runtime_error("config not read yet");
    }

    if (boost::optional<T> value = Config::_pt.get_optional<T>(key)) {
      return value.get();
    } else {
      throw std::runtime_error("config reading error");
    }
  }

  std::string get_config_all() {
    std::stringstream ss;
    boost::property_tree::write_ini(ss, _pt);
    return ss.str();
  }

  static boost::property_tree::ptree _pt;

 private:
  Config() = default;
  ~Config() = default;
};

/** static 変数は初期化が必要 */
boost::property_tree::ptree Config::_pt = boost::property_tree::ptree();

/**
 * std::vectorは非virtual destructorを持つためprotected継承する
 * cf. https://mahou-ptr.hatenablog.com/entry/2017/12/05/163120
 */
template <class T>
class PBar : protected std::vector<T> {
 public:
  void print() {
    for (const auto& elem : this) {
      std::cout << elem << std::endl;
    }
    return;
  }
};

}  // namespace util
