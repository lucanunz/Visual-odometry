#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>

typedef Eigen::Matrix<float, 10, 1> Vector10f;
typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
typedef std::vector<Vector10f, Eigen::aligned_allocator<Vector10f> > Vector10fVector;