#pragma once

#include <boost/optional.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>

namespace utils {
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
    // インスタンス破棄時に結果出力
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

  /** 設定ファイルの内容全てを string に出力 */
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
}  // namespace utils

namespace utils {
/** static 変数は初期化が必要 */
boost::property_tree::ptree Config::_pt = boost::property_tree::ptree();

void print_gauge(double x) {
  std::cout << std::setfill('0') << std::right << std::setw(2) << int(x * 100)
            << "%|";
  for (uint i = 0; i < 20; ++i) {
    if (double(i) / 20. < x) {
      std::cout << "█";
    } else {
      std::cout << " ";
    }
  }
  std::cout << "|" << std::endl;
}

/**
 * cf. https://gist.github.com/jeetsukumaran/307264
 */
template <typename T>
class PBar {
 public:
  class iterator {
   public:
    iterator(T* ptr, int size) : _ptr(ptr), _size(size) {}

    T& operator*() { return *_ptr; }
    iterator operator++() {
      iterator i = *this;
      _ptr++;
      print_gauge(double(_counter) / double(_size));
      const auto curr_time = std::chrono::system_clock::now();

      _pre_time = curr_time;
      _counter++;
      return i;
    }
    iterator operator++(int junk) {
      _ptr++;

      const auto curr_time = std::chrono::system_clock::now();
      _pre_time = curr_time;
      _counter++;
      return *this;
    }
    bool operator==(const iterator& rhs) { return _ptr == rhs._ptr; }
    bool operator!=(const iterator& rhs) { return _ptr != rhs._ptr; }

   private:
    T* _ptr;
    int _size;
    int _counter{0};
    std::chrono::system_clock::time_point _pre_time;
    std::time_t total_time{0};
  };

  PBar(int size) : _size(size) { _data = new T[_size]; }
  PBar(const std::vector<T>& vec) : _size(vec.size()) {
    _data = new T[_size];
    for (int i = 0; i < _size; i++) _data[i] = vec[i];
  }
  iterator begin() { return iterator(_data, _size); }
  iterator end() { return iterator(_data + _size, _size); }

 private:
  T* _data;
  int _size;
};

}  // namespace utils
