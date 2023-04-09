/***
 * from http://ankokudan.org/d/dl/pdf/pdf-eigennote.pdf
 */

#include <Eigen/Dense>
#include <iostream>

namespace Eigen {
// 行優先メモリ確保行列
template <typename T>
using RMatrix =
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RMatrixXd = RMatrix<double>;
}  // namespace Eigen

int main() {
  Eigen::MatrixXd A(2, 2);
  {
    A << 1, 2, 3, 4;
    std::cout << "A (Case 1):" << std::endl << A << std::endl << std::endl;
  }

  // Eigen::MatrixXd -> std::vector<double>
  {
    std::vector<double> vector(4);
    // 行優先で格納
    Eigen::Map<Eigen::RMatrixXd>(vector.data(), 2, 2) = A;

    std::cout << "transferred vector (Case 2):" << std::endl;
    for (uint i = 0; i < 4; ++i) std::cout << vector[i] << ",";
    std::cout << std::endl << std::endl;
  }

  // std::vector<double> -> Eigen::MatrixXd
  {
    std::vector<double> vector = {4, 3, 2, 1};
    // 行優先で格納
    A = Eigen::Map<Eigen::RMatrixXd>(vector.data(), 2, 2);
    std::cout << "A (Case 3):" << std::endl << A << std::endl << std::endl;
  }

  // Eigen::MatrixXd -> double**
  {
    A << 1, 2, 3, 4;
    double vector[2][2];
    Eigen::Map<Eigen::RMatrixXd>(&(vector[0][0]), 2, 2) = A;

    std::cout << "transferred vector (Case 4):" << std::endl;
    for (uint i = 0; i < 2; ++i)
      for (uint j = 0; j < 2; ++j) std::cout << vector[i][j] << ",";
    std::cout << std::endl << std::endl;
  }

  // std::vector<std::vector<<double>> -> Eigen::MatrixXd
  {
    std::vector<std::vector<double>> vector = {{1, 2}, {2, 3}};
    for (uint i = 0; i < 2; ++i)
      A.row(i) = Eigen::Map<Eigen::VectorXd>(vector[i].data(), 2);
    std::cout << "A (Case 5):" << std::endl << A << std::endl << std::endl;
  }

  // 対角化
  {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(A);
    if (es.info() != Eigen::Success) abort();
    std::cout << "eigenvalues:" << std::endl
              << es.eigenvalues() << std::endl
              << std::endl;
    std::cout << "eigenvectors:" << std::endl
              << es.eigenvectors() << std::endl
              << std::endl;
  }
}