/**
 * Base program: https://github.com/gaoxiang12/slambook2
 * Autodiff sample:
 * https://github.com/RainerKuemmerle/g2o/blob/20230223_git/g2o/examples/data_fitting/curve_fit.cpp
 * API reference by third party:
 * http://docs.ros.org/en/melodic/api/orb_slam2_ros/html/namespaceg2o.html
 */
// For macro to use autodiff
#include <g2o/core/auto_differentiation.h>
// For base class of edge
#include <g2o/core/base_unary_edge.h>
// For base class of Vertex
#include <g2o/core/base_vertex.h>
// For specifying optimization method
#include <g2o/core/optimization_algorithm_factory.h>
// For solver
#include <g2o/core/sparse_optimizer.h>
#include <matplotlibcpp.h>

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>

#include "utils.h"

// For g2o::OptimizationAlgorithmFactory
// cf. https://github.com/RainerKuemmerle/g2o/issues/496#issuecomment-791897107
G2O_USE_OPTIMIZATION_LIBRARY(dense);

// 頂点：最適化パラメータの次元・型(EstimateType)
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CurveFittingVertex() = default;

  // リセット処理
  void setToOriginImpl() override { _estimate << 0, 0, 0; }

  // 更新処理
  void oplusImpl(const double *update) override {
    _estimate += Eigen::Vector3d(update);
  }

  // ダミー関数
  bool read(std::istream &in) override { return true; }

  // ダミー関数
  bool write(std::ostream &out) const override { return true; }
};

// 辺：接続する頂点の数(EdgeDimension)・測定の型(Measurement)・頂点の型(VertexXiType)
class CurveFittingEdge
    : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, CurveFittingVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CurveFittingEdge() = default;

  template <typename T>
  bool operator()(const T *params, T *error) const {
    const T &a = params[0];
    const T &b = params[1];
    const T &c = params[2];
    const T &x = T(measurement()(0));
    const double &y = measurement()(1);

    T fval = exp(a * x * x + b * x + c);
    error[0] = fval - y;
    return true;
  }

  // ダミー関数
  bool read(std::istream &in) override { return true; }

  // ダミー関数
  bool write(std::ostream &out) const override { return true; }

  // 自動微分を使用
  G2O_MAKE_AUTO_AD_FUNCTIONS
};

int main() {
  // ガウシアンノイズの標準偏差
  constexpr double w_sigma = 1.0;
  // サンプリングデータ
  std::vector<double> x_data, y_true_data, y_obs_data;
  {
    // サンプリング数
    constexpr uint N = 100;
    // パラメータの正解
    constexpr double ar = 1.0, br = 2.0, cr = 1.0;
    // 乱数生成器
    cv::RNG rng;
    // データ生成
    for (uint i = 0; i < N; i++) {
      const double x = i / 100.0;
      x_data.push_back(x);
      const double y = std::exp(ar * x * x + br * x + cr);
      y_true_data.push_back(y);
      y_obs_data.push_back(y + rng.gaussian(w_sigma * w_sigma));
    }
  }

  // Solver の登録
  g2o::SparseOptimizer solver;
  // デバッグ情報有効化
  solver.setVerbose(true);
  // 最適化アルゴリズム設定
  {
    g2o::OptimizationAlgorithmProperty solverProperty;
    // Gauss-Newton 法での最適化アルゴリズム
    auto opt_algorithm =
        g2o::OptimizationAlgorithmFactory::instance()->construct(
            "lm_dense", solverProperty);
    solver.setAlgorithm(opt_algorithm);
  }

  // 推定対象(頂点)
  auto *v = new CurveFittingVertex();
  {
    // パラメータの初期推定値
    constexpr double ae = 2.0, be = -1.0, ce = 5.0;

    // 頂点の ID 設定
    v->setId(0);
    // 頂点に推定値を設定
    v->setEstimate(Eigen::Vector3d(ae, be, ce));
    // ソルバに頂点登録
    solver.addVertex(v);
  }

  // 測定結果(辺登録)
  for (uint i = 0; i < x_data.size(); i++) {
    auto *edge = new CurveFittingEdge();

    // 辺に ID 設定
    edge->setId(i);
    // 辺の 0 番目の頂点が v
    edge->setVertex(0, v);

    // 辺に測定値登録
    edge->setMeasurement(Eigen::Vector2d(x_data[i], y_obs_data[i]));
    // 情報行列（誤差共分散行列の逆行列）
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() /
                         (w_sigma * w_sigma));
    // ソルバに辺登録
    solver.addEdge(edge);
  }

  // 最適化実施
  {
    util::Timer("Optimization");
    solver.initializeOptimization();
    const uint max_iter_num = 10;
    solver.optimize(max_iter_num);
  }

  // 推定結果
  const Eigen::Vector3d abc_est = v->estimate();
  std::cout << "estimated model: " << abc_est.transpose() << std::endl;

  // 結果の描画
  {
    namespace plt = matplotlibcpp;

    // プロット1: 測定データ(散布図)
    {
      // 実装：https://github.com/lava/matplotlib-cpp/blob/ef0383f1315d32e0156335e10b82e90b334f6d9f/matplotlibcpp.h#L993
      constexpr double s = 2.0;
      std::map<std::string, std::string> options = {{"c", "gray"},
                                                    {"label", "measurements"}};
      plt::scatter(x_data, y_obs_data, s, options);
    }

    // プロット2：正解(線形補間)
    {
      // 実装：https://github.com/lava/matplotlib-cpp/blob/ef0383f1315d32e0156335e10b82e90b334f6d9f/matplotlibcpp.h#L442
      // 色一覧：https://matplotlib.org/stable/tutorials/colors/colors.html
      std::map<std::string, std::string> options = {{"c", "tab:blue"},
                                                    {"label", "true form"}};
      plt::plot(x_data, y_true_data, options);
    }

    // プロット3: 推定結果(線形補間)
    {
      auto y_est = [abc_est](double x) {
        return exp(abc_est[0] * x * x + abc_est[1] * x + abc_est[2]);
      };
      std::vector<double> y_est_data;
      for (const auto &x : x_data) y_est_data.push_back(y_est(x));

      std::map<std::string, std::string> options = {
          {"c", "tab:orange"}, {"label", "estimated form"}};
      plt::plot(x_data, y_est_data, options);
    }

    plt::legend();
    plt::show();
  }

  return 0;
}