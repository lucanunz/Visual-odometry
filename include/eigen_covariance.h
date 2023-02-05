#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

template <typename Iterator_>
int computeMeanAndCovariance(Eigen::Matrix<typename Iterator_::value_type::Scalar,
                                           Iterator_::value_type::RowsAtCompileTime-1, 1>& mean,
                             Eigen::Matrix<typename Iterator_::value_type::Scalar,
                                           Iterator_::value_type::RowsAtCompileTime-1,
                                           Iterator_::value_type::RowsAtCompileTime-1>& cov,
                             Iterator_ begin,
                             Iterator_ end) {
  // mean computed as 1/(end-start) Sum_k={start..end} x_k
  // cov computed as  (1/(end-start) Sum_k={start..end} x_k* x_k^T ) - mean*mean.transpose();
  using Scalar=typename Iterator_::value_type::Scalar;
  mean.setZero();
  cov.setZero();
  int k=0;
  for (auto it=begin; it!=end; ++it) {
    const auto& v=(*it).tail(Iterator_::value_type::RowsAtCompileTime-1);
    mean += v;
    cov  += v * v.transpose();
    ++k;
  }
  mean *= (1./k);
  cov  *= (1./k);
  cov  -= mean * mean.transpose();
  cov  *= Scalar(k)/Scalar(k-1);
  return k;
}

// this invokes the Eigen functions to compute the eigenvalues of
// a symmetric matrix. We extract the largest EigenVector of the covariance
// which denotes the direction of highest variation.
template <typename SquareMatrixType_>
Eigen::Matrix<typename SquareMatrixType_::Scalar,
              SquareMatrixType_::RowsAtCompileTime, 1>

largestEigenVector(const SquareMatrixType_& m) {
  Eigen::SelfAdjointEigenSolver<SquareMatrixType_> es;
  es.compute(m);
  return es.eigenvectors().col(SquareMatrixType_::RowsAtCompileTime-1);
}
