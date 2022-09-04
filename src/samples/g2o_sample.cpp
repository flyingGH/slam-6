/**
 * from https://github.com/gaoxiang12/slambook2
 */
#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>
#include <matplotlibcpp.h>

using namespace std;

// 頂点：最適化パラメータの次元・型(EstimateType)
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // リセット処理
    void setToOriginImpl() override {
        _estimate << 0, 0, 0;
    }

    // 更新処理
    void oplusImpl(const double *update) override {
        _estimate += Eigen::Vector3d(update);
    }

    // ダミー関数
    bool read(istream &in) override { return true; }

    // ダミー関数
    bool write(ostream &out) const override { return true; }
};

// 辺：接続する頂点の数(EdgeDimension)・測定の型(Measurement)・頂点の型(VertexXiType)
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

    // 残差の定義
    void computeError() override {
        const auto *v = dynamic_cast<const CurveFittingVertex *> (_vertices[0]);
        // 推定したパラメータ
        const Eigen::Vector3d abc_est = v->estimate();
        _error(0, 0) = _measurement - std::exp(abc_est[0] * _x * _x + abc_est[1] * _x + abc_est[2]);
    }

    // ヤコビ行列（のマイナス）
    void linearizeOplus() override {
        const auto *v = dynamic_cast<const CurveFittingVertex *> (_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
        _jacobianOplusXi[0] = -_x * _x * y;
        _jacobianOplusXi[1] = -_x * y;
        _jacobianOplusXi[2] = -y;
    }

    // ダミー関数
    bool read(istream &in) override { return true; }

    // ダミー関数
    bool write(ostream &out) const override { return true; }

public:
    double _x;  // x 值， y 值为 _measurement
};

int main() {
    // サンプリング数
    uint16_t N = 100;
    // ガウシアンノイズの標準偏差
    double w_sigma = 1.0;
    // サンプリングデータ
    vector<double> x_data, y_true_data, y_obs_data;
    {
        // パラメータの正解
        double ar = 1.0, br = 2.0, cr = 1.0;
        // 乱数生成器
        cv::RNG rng;
        // データ生成
        for (uint16_t i = 0; i < N; i++) {
            double x = i / 100.0;
            x_data.push_back(x);
            y_true_data.push_back(exp(ar * x * x + br * x + cr));
            y_obs_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
        }
    }

    g2o::SparseOptimizer solver;
    {
        // 状態変数は3つ(PoseDim)・観測値は1つ(LandmarkDim)
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;
        // 線形ソルバ設定
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        // Gauss-Newton 法での最適化アルゴリズム
        auto opt_algorithm = new g2o::OptimizationAlgorithmGaussNewton(
                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        // 最適化アルゴリズム設定
        solver.setAlgorithm(opt_algorithm);
        // デバッグ情報有効化
        solver.setVerbose(true);
    }

    // 頂点
    auto *v = new CurveFittingVertex();
    {
        // 推定したパラメータ
        double ae = 2.0, be = -1.0, ce = 5.0;
        // 頂点に推定値を設定
        v->setEstimate(Eigen::Vector3d(ae, be, ce));
        // 頂点の ID 設定
        v->setId(0);
        // ソルバに頂点登録
        solver.addVertex(v);
    }

    // 辺登録
    for (uint16_t i = 0; i < N; i++) {
        auto *edge = new CurveFittingEdge(x_data[i]);
        // 辺に ID 設定
        edge->setId(i);
        // 辺の 0 番目の頂点が v
        edge->setVertex(0, v);
        // 辺に測定値登録
        edge->setMeasurement(y_obs_data[i]);
        // 情報行列（誤差共分散行列の逆行列）
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));
        // ソルバに辺登録
        solver.addEdge(edge);
    }

    {
        cout << "start optimization" << endl;
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        solver.initializeOptimization();
        // 繰り返し回数10回
        solver.optimize(10);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
    }

    // 推定結果
    Eigen::Vector3d abc_est = v->estimate();
    cout << "estimated model: " << abc_est.transpose() << endl;


    {
        namespace plt = matplotlibcpp;

        // 散布図のフォーマット
        map<string, string> scatter_map;
        scatter_map["c"] = "gray";
        // 測定データ
        plt::scatter(x_data, y_obs_data, 2.0, scatter_map);

        // 答え
        // cf. https://matplotlib.org/stable/tutorials/colors/colors.html
        plt::plot(x_data, y_true_data, "tab:blue");

        // 推定結果
        vector<double> y_est_data;
        for (const auto &x: x_data) {
            y_est_data.push_back(exp(abc_est[0] * x * x + abc_est[1] * x + abc_est[2]));
        }
        plt::plot(x_data, y_est_data, "tab:orange");

        plt::show();
    }

    return 0;
}