/**
 * from https://github.com/gaoxiang12/slambook2
 */
#include <ceres/ceres.h>
#include <matplotlibcpp.h>

#include <chrono>
#include <iostream>
#include <opencv2/core/core.hpp>

#include "utils.h"

// コスト関数
struct polynomial {
  polynomial(double x, double y) : _x(x), _y(y) {}

  /** 残差の定義（自働微分のためテンプレート化）
   * @tparam T double 型 or ceres::jet 型
   * @param abc 最適化パラメータ
   * @param residual 残差
   */
  template <typename T>
  bool operator()(const T *const abc, T *residual) const {
    const T &a = abc[0];
    const T &b = abc[1];
    const T &c = abc[2];
    const T x = T(_x);
    const T y = T(_y);

    residual[0] = y - ceres::exp(a * x * x + b * x + c);
    return true;
  }

  // 測定値
  const double _x, _y;
};

int main() {
  // サンプリングダータ
  std::vector<double> x_data, y_true_data, y_obs_data;
  {
    // サンプリング数
    constexpr uint N = 100;
    // パラメータの正解
    constexpr double ar = 1.0, br = 2.0, cr = 1.0;
    // ガウシアンノイズの標準偏差
    constexpr double w_sigma = 1.0;
    // 乱数生成器
    cv::RNG rng;
    for (uint i = 0; i < N; i++) {
      const double x = i / 100.0;
      x_data.push_back(x);
      const double y = exp(ar * x * x + br * x + cr);
      y_true_data.push_back(y);
      y_obs_data.push_back(y + rng.gaussian(w_sigma * w_sigma));
    }
  }

  constexpr double ae = 2.0, be = -1.0, ce = 5.0;
  double abc[3] = {ae, be, ce};

  ceres::Solver::Summary summary;
  {
    // 最適化タスク
    ceres::Problem problem;
    for (uint i = 0; i < x_data.size(); i++) {
      // コスト関数は残差・損失関数は最適化の目標関数(残差平方和等)
      // http://ceres-solver.org/nnls_modeling.html?highlight=addresidualblock#introduction
      problem.AddResidualBlock(
          // コスト関数は polynomial・関数の引数は1・最適化パラメータは3
          new ceres::AutoDiffCostFunction<polynomial, 1, 3>(
              new polynomial(x_data[i], y_obs_data[i])),
          // 損失関数はデフォルトでは残差平方和
          nullptr,
          // 最適化の初期値
          abc);
    }

    // 最適化ソルバーを指定
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;

    utils::Timer("Optimization");
    // タスクとソルバーを指定して解く
    ceres::Solve(options, &problem, &summary);
  }

  // 最適化結果
  {
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "estimated a,b,c = ";
    for (const auto &a : abc) std::cout << a << " ";
    std::cout << std::endl;
  }

  // 結果の描画
  {
    namespace plt = matplotlibcpp;

    // 測定値(散布図)
    {
      constexpr double s = 2.0;
      std::map<std::string, std::string> options = {{"c", "gray"},
                                                    {"label", "measurements"}};
      plt::scatter(x_data, y_obs_data, s, options);
    }

    // 答え
    {
      // cf. https://matplotlib.org/stable/tutorials/colors/colors.html
      std::map<std::string, std::string> options = {{"c", "tab:blue"},
                                                    {"label", "true form"}};
      plt::plot(x_data, y_true_data, options);
    }

    // 推定結果
    {
      std::vector<double> y_est_data;
      for (const auto &x : x_data)
        y_est_data.push_back(exp(abc[0] * x * x + abc[1] * x + abc[2]));

      std::map<std::string, std::string> options = {
          {"c", "tab:orange"}, {"label", "estimated form"}};
      plt::plot(x_data, y_est_data, options);
    }

    plt::title("Curve fitting by ceres-solver");
    plt::legend();
    plt::show();
  }

  return 0;
}