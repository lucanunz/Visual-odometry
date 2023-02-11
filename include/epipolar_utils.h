#include "defs.h"
#include "utils.h"

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

//! Normalizes the coordinates of the points in p to be between -1 and 1
//! @param p: points to normalize
//! @param T: will contain the preconditioning matrix
//! @returns the normalized points
Vector2fVector normalize(const Vector2fVector& p, Eigen::Matrix3f& T);    

//! Not used
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
