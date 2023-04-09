#include "utils.h"

#include <filesystem>

using utils::Config;

int main() {
  const std::string config_path =
      std::filesystem::current_path().append("config.ini").string();
  Config::read_ini(config_path);
  const auto hoge = Config::get_config<uint>("test.hoge");
  const auto fuga = Config::get_config<std::string>("test.fuga");

  // const auto pbar = utils::PBar<double>({1, 2, 3, 4, 5});

  return 0;
}