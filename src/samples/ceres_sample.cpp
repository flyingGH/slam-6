/**
 * from https://github.com/gaoxiang12/slambook2
 */
#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include <matplotlibcpp.h>

using namespace std;

// コスト関数
struct polynomial {
    polynomial(double x, double y) : _x(x), _y(y) {}

    /** 残差の定義（自働微分のためテンプレート化）
     * @tparam T double 型 or ceres::jet 型
     * @param abc 最適化パラメータ
     * @param residual 残差
     */
    template<typename T>
    bool operator()(const T *const abc,T *residual) const {
        // y-exp(ax^2+bx+c)
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
        return true;
    }

    // 測定値
    const double _x, _y;
};

int main() {
    // サンプリング数
    uint16_t N = 100;
    // サンプリングダータ
    vector<double> x_data, y_true_data, y_obs_data;
    {
        // パラメータの正解
        double ar = 1.0, br = 2.0, cr = 1.0;
        // ガウシアンノイズの標準偏差
        double w_sigma = 1.0;
        // 乱数生成器
        cv::RNG rng;
        for (int i = 0; i < N; i++) {
            double x = i / 100.0;
            x_data.push_back(x);
            y_true_data.push_back(exp(ar * x * x + br * x + cr));
            y_obs_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
        }
    }

    double ae = 2.0, be = -1.0, ce = 5.0;
    double abc[3] = {ae, be, ce};

    ceres::Solver::Summary summary;
    {
        // 最適化タスク
        ceres::Problem problem;
        for (uint16_t i = 0; i < N; i++) {
            problem.AddResidualBlock(
                    // 目的関数は polynomial・関数の引数は1・最適化パラメータは3
                    new ceres::AutoDiffCostFunction<polynomial, 1, 3>(
                            new polynomial(x_data[i], y_obs_data[i])
                    ),
                    nullptr,
                    // 最適化の初期値
                    abc
            );
        }

        // 最適化の方法
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = true;

        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        ceres::Solve(options, &problem, &summary);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
    }

    // 最適化結果
    cout << summary.BriefReport() << endl;
    cout << "estimated a,b,c = ";
    for (auto a:abc) cout << a << " ";
    cout << endl;

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
            y_est_data.push_back(exp(abc[0] * x * x + abc[1] * x + abc[2]));
        }
        plt::plot(x_data, y_est_data, "tab:orange");

        plt::show();
    }

    return 0;
}