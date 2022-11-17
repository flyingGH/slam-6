/**
 * from https://github.com/gaoxiang12/slambook2
 */
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

/// 本程序演示sophus的基本用法

int main() {
    // 元となる回転行列（回転方向と回転量を指定）
    Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
    // 回転行列表現 -> SO(3)
    Sophus::SO3d SO3_R(R);
    // 回転行列表現 -> 四元数表現
    Quaterniond q(R);
    {
        // 四元数からSO(3)の元に変換
        Sophus::SO3d SO3_q(q);

        // 四元数を介しても結果は同じ
        cout << "Original matrix:\n" << R << "\n" << endl;
        cout << "SO(3) from matrix:\n" << SO3_R.matrix() << "\n" << endl;
        cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << "\n" <<  endl;
    }

    {
        // 対数写像：リー群の元 -> リー代数 (の成分表示・ベクトル表現)
        Vector3d so3 = SO3_R.log();
        cout << "so3 = " << so3.transpose() << "\n" << endl;

        // リー代数のベクトル表現 -> 行列表現
        // cf. https://strasdat.github.io/Sophus/class_Sophus_SO3.html
        cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << "\n" << endl;
        // リー代数の行列表現 -> ベクトル表現
        cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << "\n" << endl;
    }

    {
        // リー代数の摂動
        Vector3d update_so3(1e-4, 0, 0);
        // リー群の摂動
        Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
        cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;
    }

    cout << "*******************************" << endl;

    // 並進ベクトル
    Vector3d t(1, 0, 0);
    // (回転行列, 並進ベクトル) -> SE(3)
    Sophus::SE3d SE3_Rt(R, t);
    {
        // (四元数, 並進ベクトル) -> SE(3)
        Sophus::SE3d SE3_qt(q, t);
        cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << "\n" << endl;
        cout << "SE3 from q,t= \n" << SE3_qt.matrix() << "\n" << endl;
    }

    // 6次元縦ベクトルを定義
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    {
        // 対数写像：リー群の元 -> リー代数 (の成分表示・ベクトル表現)
        Vector6d se3 = SE3_Rt.log();
        cout << "se3 = " << se3.transpose() << "\n" << endl;

        // リー代数のベクトル表現 -> 行列表現
        cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << "\n" << endl;
        // リー代数の行列表現 -> ベクトル表現
        cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << "\n" << endl;
    }

    // 摂動
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;

    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;

    return 0;
}