#include "picp_solver.h"

#include <Eigen/Cholesky>
#include <iostream>

  PICPSolver::PICPSolver()
  {
    _world_points = 0;
    _reference_image_points = 0;
    _damping = 1;
    _min_num_inliers = 0;
    _num_inliers = 0;
    _kernel_thereshold = 1000; // 33 pixels
  }

  void PICPSolver::init(const Camera &camera_,
                        const Vector3fVector &world_points,
                        const Vector2fVector &reference_image_points)
  {
    _camera = camera_;
    _world_points = &world_points;
    _reference_image_points = &reference_image_points;
  }

  bool PICPSolver::errorAndJacobian(Eigen::Vector2f &error,
                                    Matrix2_3f &jacobian,
                                    const Eigen::Vector3f &world_point,
                                    const Eigen::Vector2f &reference_image_point)
  {
    // compute the prediction
    Eigen::Vector2f predicted_image_point;
    bool is_good = _camera.projectPoint(predicted_image_point, world_point);
    if (!is_good)
      return false;
    error = predicted_image_point - reference_image_point;

    // compute the jacobian of the transformation
    Eigen::Vector3f camera_point = _camera.worldInCameraPose() * world_point;
    Eigen::Matrix3f Jr = Eigen::Matrix3f::Zero();
    Jr << 1,0,camera_point.z(),
          0,0,0,
          0,1,-camera_point.x();

    Eigen::Vector3f phom = _camera.cameraMatrix() * camera_point;
    float iz = 1. / phom.z();
    float iz2 = iz * iz;
    // jacobian of the projection
    Matrix2_3f Jp;
    Jp << iz, 0, -phom.x() * iz2,
        0, iz, -phom.y() * iz2;

    jacobian = Jp * _camera.cameraMatrix() * Jr;
    return true;
  }

  void PICPSolver::linearize(const IntPairVector &correspondences, bool keep_outliers)
  {
    _H.setZero();
    _b.setZero();
    _num_inliers = 0;
    _chi_inliers = 0;
    _chi_outliers = 0;
    for (const IntPair &correspondence : correspondences)
    {
      Eigen::Vector2f e;
      Matrix2_3f J;
      int ref_idx = correspondence.first;
      int curr_idx = correspondence.second;
      bool inside = errorAndJacobian(e,
                                     J,
                                     (*_world_points)[curr_idx],
                                     (*_reference_image_points)[ref_idx]);
      if (!inside)
        continue;

      float chi = e.dot(e);
      float lambda = 1;
      bool is_inlier = true;
      if (chi > _kernel_thereshold)
      {
        lambda = sqrt(_kernel_thereshold / chi);
        is_inlier = false;
        _chi_outliers += chi;
      }
      else
      {
        _chi_inliers += chi;
        _num_inliers++;
      }

      if (is_inlier || keep_outliers)
      {
        _H += J.transpose() * J * lambda;
        _b += J.transpose() * e * lambda;
      }
    }
  }

  bool PICPSolver::oneRound(const IntPairVector &correspondences, bool keep_outliers)
  {
    using namespace std;
    linearize(correspondences, keep_outliers);
    _H += Eigen::Matrix3f::Identity() * _damping;
    if (_num_inliers < _min_num_inliers)
    {
      cerr << "too few inliers, skipping" << endl;
      return false;
    }
    // compute a solution
    Eigen::Vector3f dx = _H.ldlt().solve(-_b);
    _camera.setWorldInCameraPose(v2tEuler((Vector6f() << dx.x(),0.f,dx.y(),0.f,dx.z(),0.f).finished()) * _camera.worldInCameraPose());
    return true;
  }
