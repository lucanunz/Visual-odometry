#pragma once
#include <unordered_set>
#include <iostream>
#include <Eigen/Eigenvalues>
#include <random>
#include "defs.h"

//! computes the rotation matrix around the x axis
//! @param angle: angle of rotation
//! @returns the rotation matrix
template <typename Scalar_>
Eigen::Matrix<Scalar_,3,3> RotationX(const Scalar_& angle) {
  Eigen::Matrix<Scalar_,3,3> R;
  const Scalar_ s=sin(angle);
  const Scalar_ c=cos(angle);
  const Scalar_ one(1.);
  const Scalar_ zero(0.);
  R << 
    one, zero, zero,
    zero, c, -s,
    zero, s, c;
  return R;
}
//! computes the rotation matrix around the y axis
//! @param angle: angle of rotation
//! @returns the rotation matrix
template <typename Scalar_>
Eigen::Matrix<Scalar_,3,3> RotationY(const Scalar_& angle) {
  Eigen::Matrix<Scalar_,3,3> R;
  const Scalar_ s=sin(angle);
  const Scalar_ c=cos(angle);
  const Scalar_ one(1.);
  const Scalar_ zero(0.);
  R << 
    c, zero, s,
    zero, one, zero,
    -s, zero, c;
  return R;
}
//! computes the rotation matrix around the z axis
//! @param angle: angle of rotation
//! @returns the rotation matrix
template <typename Scalar_>
Eigen::Matrix<Scalar_,3,3> RotationZ(const Scalar_& angle) {
  Eigen::Matrix<Scalar_,3,3> R;
  const Scalar_ s=sin(angle);
  const Scalar_ c=cos(angle);
  const Scalar_ one(1.);
  const Scalar_ zero(0.);
  R << 
    c, -s, zero,
    s, c, zero,
    zero, zero, one;
  return R;
}
//! computes a rotation matrix given xyz euler angles
//! @param angle: angles of rotation
//! @returns the rotation matrix
template <typename Scalar_>
Eigen::Matrix<Scalar_,3,3> Rotation(const Eigen::Matrix<Scalar_,3,1>& angles) {
  return RotationX(angles.x())*RotationY(angles.y())*RotationZ(angles.z());
}

//! computes an isometry from the 6dim vector (x y z th_x th_y th_z)
//! where the angles are considered to be euler angles
//! @param v: 6dim vector
//! @returns the computed isometry
inline Eigen::Isometry3f v2tEuler(const Vector6f& v){
  Eigen::Isometry3f T;
  T.linear()=Rotation(Eigen::Vector3f(v.tail<3>()));
  T.translation()=v.head<3>();
  return T;
}

//! computes the eigenvector associated to the smallest eigenvalue of a given matrix
//! @param m: matrix of which to compute the eigenvectors
//! @returns the eigenvector associated to the smallest eigenvalueof m
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
inline Eigen::Matrix3f skew(const Eigen::Vector3f& v){
  Eigen::Matrix3f ret;
  ret << 0,-v(2),v(1),
      v(2),0,-v(0),
      -v(1),v(0),0;
  return ret;
}

//! generates a random homogeneous transformation in 3D
//! @param X: isometry3f where to write the random transformation
void generate_isometry3f(Eigen::Isometry3f& X);

//! generates a set of random points in the 3D space.
//! @param num_points: number of points to generate
//! @returns the generated set of points
Vector3fVector generate_points3d(const int& num_points);

//! returns the ids only of the points that are present in both sets. The sets contains 3D projected vectors: id-col-row.
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

//! Not used
//! estimate essential matrix from two sets of corresponding points.
//! @param k: 3x3 camera matrix
//! @param correspondences: correspondences (first: idx of the point in the first image, second: idx of the corresponding point in the second image)
//! @param p1_img: points in the first image
//! @param p2_img: points in the second image
//! @returns the essential matrix
const Eigen::Matrix3f estimate_essential(const Eigen::Matrix3f& k, const IntPairVector& correspondences, 
                                          const Vector2fVector& p1_img, const Vector2fVector& p2_img);

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
//! @param p: will containt the intersection point
//! @returns false if the triangulated point falls behind one of the cameras. In that case, the value of p is invalid.
bool triangulate_point(const Eigen::Vector3f& d1,const Eigen::Vector3f& d2,const Eigen::Vector3f& p2,Eigen::Vector3f& p);

//! triangulate points given their projections on two images, and the relative pose between the cameras
//! @param k: 3x3 camera matrix
//! @param X: relative pose of the first camera expressed in the frame of the second
//! @param correspondences: correspondences (first: idx of the point in the first image, second: idx of the corresponding point in the second image)
//! @param p1_img: points in the first image
//! @param p2_img: points in the second image
//! @param triangulated: this vector will contain the triangulated points
//! @returns the number of successfully triangulated points
int triangulate_points(const Eigen::Matrix3f& k, const Eigen::Isometry3f& X, const IntPairVector& correspondences,
                        const Vector2fVector& p1_img, const Vector2fVector& p2_img, Vector3fVector& triangulated);

//! @overload
//! triangulate points given their projections on two images, and the relative pose between the cameras. Computes also
//! a new set of correspondances between the triangulated points and the points in the second image
//! @param k: 3x3 camera matrix
//! @param X: relative pose of the first camera expressed in the frame of the second
//! @param correspondences: correspondences (first: idx of the point in the first image, second: idx of the corresponding point in the second image)
//! @param p1_img: points in the first image
//! @param p2_img: points in the second image
//! @param triangulated: this vector will contain the triangulated points
//! @param correspondences_new: this vector will contain the new correspondances (first: id of the measurement, second: id of the triangulated point)
//! @returns the number of successfully triangulated points
int triangulate_points(const Eigen::Matrix3f& k, const Eigen::Isometry3f& X, const IntPairVector& correspondences,
                        const Vector2fVector& p1_img, const Vector2fVector& p2_img, Vector3fVector& triangulated,IntPairVector& correspondences_new);

//! Normalizes the coordinates of the points in p to be between -1 and 1
//! @param p: points to normalize
//! @param T: will contain the preconditioning matrix
//! @returns the normalized points
Vector2fVector normalize(const Vector2fVector& p, Eigen::Matrix3f& T);

//! Normalizes the coordinates of the points in p to be centered in 0 and have unitary covariance
//! @param p: points to normalize
//! @param T: will contain the preconditioning matrix
//! @returns the normalized points
Vector2fVector normalizeGauss(const Vector2fVector& p,Eigen::Matrix3f& T);

//! Estimates the relative pose of the first camera expressed in the frame of the second
//! @param k: 3x3 camera matrix
//! @param correspondences: correspondences (first: idx of the point in the first image, second: idx of the corresponding point in the second image)
//! @param p1_img: points in the first image
//! @param p2_img: points in the second image
//! @returns: the most consistent pose according to the number of successfully triangulated points
const Eigen::Isometry3f estimate_transform(const Eigen::Matrix3f k, const IntPairVector& correspondences, 
                                            const Vector2fVector& p1_img, const Vector2fVector& p2_img);