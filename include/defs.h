#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>

typedef Eigen::Matrix<float, 10, 1> Vector10f;
typedef Eigen::Matrix<float, 9, 1> Vector9f;
typedef Eigen::Matrix<float, 1, 9> RowVector9f;
typedef Eigen::Matrix<float, 9, 9> Matrix9f;

typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
typedef std::vector<Vector10f, Eigen::aligned_allocator<Vector10f> > Vector10fVector;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > Vector2iVector;