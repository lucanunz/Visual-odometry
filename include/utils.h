#pragma once
#include <regex>
#include <dirent.h>
#include <set>
#include <unordered_set>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <Eigen/Eigenvalues>
#include <random>
#include "defs.h"

// function that returns the eigenvector associated to the smallest eigenval of a given matrix
template <typename SquareMatrixType_>
Eigen::Matrix<typename SquareMatrixType_::Scalar,
              SquareMatrixType_::RowsAtCompileTime, 1>

smallestEigenVector(const SquareMatrixType_& m) {
  Eigen::SelfAdjointEigenSolver<SquareMatrixType_> es;
  es.compute(m);
  return es.eigenvectors().col(0);
}

// returns the skewsymm matrix associated to v
Eigen::Matrix3f skew(const Eigen::Vector3f& v);

// writes the eigen vectors in vectors to a file, one for each row. They are to be used in gnuplot, e.g.
// splot "p1.txt" u 1:2:3 w p pt 7,"p2.txt" u 1:2:3 w p pt 7 lt rgb "#FF0000"
void write_eigen_vectors_to_file(const std::string& file_path, const Vector3fVector& vectors);

// writes in files the file names (in alphabetical order) found in path that match the regex
bool get_file_names(const std::string& path, std::set<std::string>& files,const std::regex& pattern);

// writes in appearances and features the content of the file file_path
bool get_meas_content(const std::string& path_to_file, Vector10fVector& appearances, Vector3fVector& features);

// writes in int_params z_near,z_far,rows,cols and in k the camera matrix. These data are found in the file path_to_file
bool get_camera_params(const std::string& path_to_file, std::vector<int>& int_params, Eigen::Matrix3f& k);

// returns the ids only of the points that are present in both p1 and p2
std::unordered_set<int> get_valid_ids(const Vector3fVector& p1, const Vector3fVector& p2);

// given a set of valid ids, it removes from p1 and p2 the points whose id is not valid
void prune_projections(Vector3fVector& p1, Vector3fVector& p2,const std::unordered_set<int>& ids);

// takes an isometry as input and returns the relative essential matrix
const Eigen::Matrix3f transform2essential(const Eigen::Isometry3f X);

// estimate essential matrix from two sets of corresponding points. It assumes that the i-th element of p1_img matches the i-th in p2_img
const Eigen::Matrix3f estimate_essential(const Vector3fVector& p1_img, const Vector3fVector& p2_img);

// generates a random 3d transformation
void generate_isometry3f(Eigen::Isometry3f& X);

// writes in p1 num_points randomly generated points, and then in p2 writes the same points but transformed by X
void generate_points3d(const Eigen::Isometry3f& X,const int& num_points, Vector3fVector& p1, Vector3fVector& p2);
