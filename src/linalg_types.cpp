#include <Eigen/Dense>

using Mat16x16 = Eigen::Matrix<double, 16, 16>;
using Mat1x1 = Eigen::Matrix<double, 1, 1>;
using Mat1x4 = Eigen::Matrix<double, 1, 4>;
using Mat2x2 = Eigen::Matrix<double, 2, 2>;
using Mat2x8 = Eigen::Matrix<double, 2, 8>;
using Mat4x1 = Eigen::Matrix<double, 4, 1>;
using Mat4x4 = Eigen::Matrix<double, 4, 4>;
using Mat8x2 = Eigen::Matrix<double, 8, 2>;
using Mat8x8 = Eigen::Matrix<double, 8, 8>;
using Vec1 = Eigen::Matrix<double, 1, 1>;
using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec4 = Eigen::Matrix<double, 4, 1>;
using Vec8 = Eigen::Matrix<double, 8, 1>;
using VecC16 = Eigen::Matrix<std::complex<double>, 16, 1>;
using VecC8 = Eigen::Matrix<std::complex<double>, 8, 1>;

template class Eigen::EigenSolver<Mat16x16>;
template class Eigen::Matrix<double, 16, 16>;
template class Eigen::Matrix<double, 2, 8>;
template class Eigen::Matrix<double, 8, 1>;
template class Eigen::Matrix<double, 8, 2>;
template class Eigen::Matrix<double, 8, 8>;
template class Eigen::Matrix<std::complex<double>, 16, 1>;
template class Eigen::Transpose<Mat8x8>;
