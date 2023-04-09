#include "utils.h"

#include <filesystem>

using utils::Config;

int main() {
  Config::read_ini("/workspace/cpp/samples/config.ini");
  const auto hoge = Config::get_config<uint>("test.hoge");
  std::cout << "test.hoge=" << hoge << std::endl;
  const auto fuga = Config::get_config<std::string>("test.fuga");
  std::cout << "test.fuga=" << fuga << std::endl;

  auto pbar = utils::PBar<double>({1, 2, 3, 4, 5});
  pbar.print();

  return 0;
}