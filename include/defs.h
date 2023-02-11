#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include <iostream>
typedef Eigen::Matrix<float, 11, 1> Vector11f;
typedef Eigen::Matrix<float, 10, 1> Vector10f;
typedef Eigen::Matrix<float, 9, 1> Vector9f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 1, 9> RowVector9f;
typedef Eigen::Matrix<float, 9, 9> Matrix9f;
typedef Eigen::Matrix<float, 2, 3> Matrix2_3f;
typedef Eigen::Matrix<float, 2, 6> Matrix2_6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 3, 6> Matrix3_6f;

typedef std::pair<Eigen::Isometry3f,Eigen::Isometry3f> IsometryPair;
typedef std::pair<int,int> IntPair;
typedef std::vector<IntPair > IntPairVector;

typedef std::vector<Vector11f, Eigen::aligned_allocator<Vector11f> > Vector11fVector;
typedef std::vector<Vector6f, Eigen::aligned_allocator<Vector6f> > Vector6fVector;
typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > Vector4fVector;
typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
typedef std::vector<Vector10f, Eigen::aligned_allocator<Vector10f> > Vector10fVector;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > Vector2iVector;
typedef std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > IsometryVector;