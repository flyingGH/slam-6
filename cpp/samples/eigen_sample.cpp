/***
 * from http://ankokudan.org/d/dl/pdf/pdf-eigennote.pdf
 */

#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main() {
    Matrix2f A;
    A << 1, 2, 2, 3;
    SelfAdjointEigenSolver<Matrix2f> es(A);
    if (es.info() != Success) abort();
    cout << "固有値：\n"
         << es.eigenvalues() << endl;
    cout << "固有ベクトル：\n"
         << es.eigenvectors() << endl;
}