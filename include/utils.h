#pragma once
#include <unordered_set>
#include <iostream>
#include <Eigen/Eigenvalues>
#include <random>
#include "defs.h"

//! computes the eigenvector associated to the smallest eigenvalue of a given matrix
//! @param m: matrix of which to compute the eigenvectors
//! @returns the the eigenvector associated to the smallest eigenvalueof m
template <typename SquareMatrixType_>
Eigen::Matrix<typename SquareMatrixType_::Scalar,
              SquareMatrixType_::RowsAtCompileTime, 1>

smallestEigenVector(const SquareMatrixType_& m) {
  Eigen::SelfAdjointEigenSolver<SquareMatrixType_> es;
  es.compute(m);
  return es.eigenvectors().col(0);
}

//! computes the skewsymm matrix associated to v
//! @param v: 3 dimensional vector to generate the 3x3 matrix
//! @returns the skew symmetric matrix computed from v
Eigen::Matrix3f skew(const Eigen::Vector3f& v);

//! generates a random 3d transformation
//! @param X: isometry3f where to write the random transformation
void generate_isometry3f(Eigen::Isometry3f& X);

//! generates two set of random points in the 3d space. The second is equal to the first one transformed by X
//! @param X: isometry3f that is used to transform p1, getting p2
//! @param num_points: number of points to generate
//! @param p1: first set of points
//! @param p2: second set of points
void generate_points3d(const Eigen::Isometry3f& X,const int& num_points, Vector3fVector& p1, Vector3fVector& p2);

//! returns the ids only of the points that are present in both sets. The sets contains 3d projected vectors: id-col-row.
//! @param p1_img: points in the first image, with their id
//! @param p2_img: points in the second image, with their id
//! @returns the set of valid ids
std::unordered_set<int> get_valid_ids(const Vector3fVector& p1_img, const Vector3fVector& p2_img);

//! given a set of valid ids, it removes from p1 and p2 the points whose id is not valid
//! @param p1_img: points in the first image, with their id
//! @param p2_img: points in the second image, with their id
//! @param ids: set of valid ids
void prune_projections(Vector3fVector& p1_img, Vector3fVector& p2_img,const std::unordered_set<int>& ids);

//! takes an isometry as input and returns the relative essential matrix
//! @param X: isometry3f from which compute the essential
const Eigen::Matrix3f transform2essential(const Eigen::Isometry3f X);

//! estimate essential matrix from two sets of corresponding points. It assumes that the i-th element of p1_img matches the i-th in p2_img
//! @param p1_img: points in the first image, with their id
//! @param p2_img: points in the second image, with their id
//! @param k: 3x3 camera matrix
//! @returns the essential matrix
const Eigen::Matrix3f estimate_essential(const Vector3fVector& p1_img, const Vector3fVector& p2_img,const Eigen::Matrix3f& k);

//! given the essential matrix E, estimates a pair of transformation matrix retrieved from E
//! @param E: essential matrix
//! @returns a pair of transfomations
const IsometryPair essential2transformPair(const Eigen::Matrix3f& E);

//! triangulates a point by finding the intersection of two lines:
//! one passing through the origin, and having direction d1 
//! one passing through a point p2, and having direction d2
//! @param d1: direction vector of first line
//! @param d1: direction vector of second line
//! @param p2: point on the second line
//! @param p: intersection point
//! @returns false if the triangulated point falls behind one of the cameras. In that case, the value of p is invalid.
bool triangulate_point(const Eigen::Vector3f& d1,const Eigen::Vector3f& d2,const Eigen::Vector3f& p2,Eigen::Vector3f& p);

//! triangulate points given their projections on two images, and the relative pose between the cameras
//! @param k: 3x3 camera matrix
//! @param X: relative pose of the first camera expressed in the frame of the second
//! @param p1_img: points in the first image, with their id
//! @param p2_img: points in the second image, with their id
//! @param triangulated: this vector will contain the triangulated points, together with their id
//! @returns the number of successfully triangulated points
int triangulate_points(const Eigen::Matrix3f& k, const Eigen::Isometry3f& X, const Vector3fVector& p1_img, const Vector3fVector& p2_img, Vector4fVector& triangulated);

//! given a pair of transformations estimated from the essential, it estimates the most consistent one
//! by triangulating the points on the two images
//! @param k: 3x3 camera matrix
//! @param X12: a pair of relative poses that describe the first pose expressed in the frame of the second, extracted from the essential
//! @param p1_img: points in the first image, with their id
//! @param p2_img: points in the second image, with their id
//! @returns: the most consistent pose according to the number of successfully triangulated points
const Eigen::Isometry3f most_consistent_transform(const Eigen::Matrix3f k,const IsometryPair X12,const Vector3fVector& p1_img, const Vector3fVector& p2_img);