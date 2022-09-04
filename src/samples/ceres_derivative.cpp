/**
 * from
 */
#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include <matplotlibcpp.h>

using namespace std;

struct AutoFunctor {
    AutoFunctor(double a, double b, double c, double d) : _a(a), _b(b), _c(c), _d(d) {}

    /** 多項式
     * @tparam T double or jet
     * @param x 引数
     * @param y 関数値
     */
    template<typename T>
    bool operator()(const T *const x, T *y) const {
        y[0] = T(_a) + T(_b) * x[0] + T(_c) * ceres::pow(x[0], 2) + T(_d) * ceres::pow(x[0], 3);
        return true;
    }

    // 測定値
    const double _a, _b, _c, _d;
};

struct NumericFunctor {
    NumericFunctor(double a, double b, double c, double d) : _a(a), _b(b), _c(c), _d(d) {}

    bool operator()(const double *const x, double *y) const {
        y[0] = _a + _b * x[0] + _c * ceres::pow(x[0], 2) + _d * ceres::pow(x[0], 3);
        return true;
    }

    // 測定値
    const double _a, _b, _c, _d;
};

int main() {
    double params[4]{0, 1, -2, 3};
    // operator() の戻り値の数1・xの次元1
    ceres::CostFunction *auto_func = new ceres::AutoDiffCostFunction<AutoFunctor, 1, 1>(
            new AutoFunctor(params[0], params[1], params[2], params[3]));
    ceres::CostFunction *numeric_func = new ceres::NumericDiffCostFunction<NumericFunctor, ceres::CENTRAL, 1, 1>(
            new NumericFunctor(params[0], params[1], params[2], params[3]));

    double **args;
    args = (double **) malloc(sizeof(double *) * 1);
    args[0] = (double *) malloc(sizeof(double) * 1);
    args[0][0] = 2;

    double values[1];
    double **jacobians;
    jacobians = (double **) malloc(sizeof(double *) * 1);
    jacobians[0] = (double *) malloc(sizeof(double) * 1);

    // サンプリング数
    uint16_t N = 100;
    // サンプリングダータ
    vector<double> x_data, jac_data, jac_auto_data, jac_numeric_data;
    {
        for (int i = 0; i < N; i++) {
            args[0][0] = i / 100.0;
            x_data.push_back(args[0][0]);
            jac_data.push_back(params[1] + 2 * params[2] * args[0][0] + 3 * params[3] * pow(args[0][0], 2));

            auto_func->Evaluate(args, values, jacobians);
            jac_auto_data.push_back(jacobians[0][0]);

            numeric_func->Evaluate(args, values, jacobians);
            jac_numeric_data.push_back(jacobians[0][0]);
        }
    }

    // 差分計算
    vector<double> jac_auto_diff_data, jac_numeric_diff_data;
    for (size_t idx=0; idx < x_data.size(); ++idx) {
        jac_auto_diff_data.push_back(jac_auto_data[idx] - jac_data[idx]);
        jac_numeric_diff_data.push_back(jac_numeric_data[idx] - jac_data[idx]);
    }

    {
        namespace plt = matplotlibcpp;
        plt::plot(x_data, jac_auto_diff_data, "tab:blue");
        plt::plot(x_data, jac_numeric_diff_data, "tab:orange");
        plt::show();
    }

    return 0;
}