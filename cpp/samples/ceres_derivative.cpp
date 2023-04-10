/**
 * from http://ceres-solver.org/nnls_modeling.html#autodiffcostfunction
 */
#include <ceres/ceres.h>
#include <matplotlibcpp.h>

#include <chrono>
#include <iostream>
#include <opencv2/core/core.hpp>

using namespace std;

struct AutoFunctor {
  AutoFunctor(double a, double b, double c, double d)
      : _a(a), _b(b), _c(c), _d(d) {}

  /** 多項式
   * @tparam T double or jet
   * @param x 引数
   * @param y 関数値
   */
  template <typename T>
  bool operator()(const T *x, T *y) const {
    y[0] = T(_a) + T(_b) * x[0] + T(_c) * ceres::pow(x[0], 2) +
           T(_d) * ceres::pow(x[0], 3);
    return true;
  }

  // 測定値
  const double _a, _b, _c, _d;
};

struct NumericFunctor {
  NumericFunctor(double a, double b, double c, double d)
      : _a(a), _b(b), _c(c), _d(d) {}

  bool operator()(const double *x, double *y) const {
    y[0] = _a + _b * x[0] + _c * ceres::pow(x[0], 2) + _d * ceres::pow(x[0], 3);
    return true;
  }

  // 測定値
  const double _a, _b, _c, _d;
};

int main() {
  // 関数パラメータ
  const std::vector<double> params = {0, 1, -2, 3};

  // 自動微分：operator() の戻り値の数1・xの次元1
  ceres::CostFunction *auto_func =
      new ceres::AutoDiffCostFunction<AutoFunctor, 1, 1>(
          new AutoFunctor(params[0], params[1], params[2], params[3]));
  // 数値微分：operator() の戻り値の数1・xの次元1
  ceres::CostFunction *numeric_func =
      new ceres::NumericDiffCostFunction<NumericFunctor, ceres::CENTRAL, 1, 1>(
          new NumericFunctor(params[0], params[1], params[2], params[3]));

  // x[i][j]：i番目ブロックのj番目変数
  double **x = new double *[1];
  double y[1];
  // http://ceres-solver.org/nnls_modeling.html?highlight=autodiffcostfunction#_CPPv4NK5ceres12CostFunction8EvaluateEPPCKdPdPPd
  double **dydx = new double *[1];
  {
    x[0] = new double[1];
    x[0][0] = 2;
    dydx[0] = new double[1];
  }

  // サンプリングダータ
  vector<double> x_data, jac_data, jac_auto_data, jac_numeric_data;
  {
    const auto &b = params[1];
    const auto &c = params[2];
    const auto &d = params[3];

    // サンプリング数
    constexpr uint N = 100;
    for (uint i = 0; i < N; i++) {
      x[0][0] = i / 100.0;
      x_data.push_back(x[0][0]);

      // 一階微分の正解
      jac_data.push_back(b + 2 * c * x[0][0] + 3 * d * pow(x[0][0], 2));

      // 自動微分
      auto_func->Evaluate(x, y, dydx);
      // 計算結果を保存
      jac_auto_data.push_back(dydx[0][0]);

      // 数値微分
      numeric_func->Evaluate(x, y, dydx);
      // 計算結果を保存
      jac_numeric_data.push_back(dydx[0][0]);
    }
  }

  // 差分計算
  vector<double> jac_auto_diff_data, jac_numeric_diff_data;
  for (size_t idx = 0; idx < x_data.size(); ++idx) {
    // 解析的微分と自動微分の差分
    jac_auto_diff_data.push_back(jac_auto_data[idx] - jac_data[idx]);
    // 解析的微分と数値微分の差分
    jac_numeric_diff_data.push_back(jac_numeric_data[idx] - jac_data[idx]);
  }

  {
    namespace plt = matplotlibcpp;

    std::map<std::string, std::string> options = {{"color", "tab:blue"},
                                                  {"label", "Automatic"}};
    plt::plot(x_data, jac_auto_diff_data, options);

    options = {{"color", "tab:orange"}, {"label", "Numerical"}};
    plt::plot(x_data, jac_numeric_diff_data, options);

    plt::title("Residual with analytic derivatives");

    plt::xlabel("$x$");
    plt::ylabel(R"($\Delta\left(\dfrac{d}{dx}(a+bx+cx^2+dx^3)\right)$)");
    plt::legend();
    plt::show();
  }

  return 0;
}