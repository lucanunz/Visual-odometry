#pragma once
#include "defs.h"

  /**
     simple pinhole camera class.
     Has
     - the position (world with respect to camera)
     - z_near/z_far how close/far the camera can perceive stuff
     - the size of the image plane in pixels
     Supports simple projection operations
  */
  class Camera{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    //~ ctor, initialized a camera according to the arguments
    Camera(int rows=100,
           int cols=100,
           int z_near=0,
           int z_far=10,
           const Eigen::Matrix3f& camera_matrix=Eigen::Matrix3f::Identity(),
           const Eigen::Isometry3f& world_in_camera_pose=Eigen::Isometry3f::Identity());

    // ! projects a single point on the image plane
    // ! @returns false if the point is behind the camera or outside the plane
    inline  bool projectPoint(Eigen::Vector2f& image_point,
                              const Eigen::Vector3f& world_point){
      Eigen::Vector3f camera_point=_world_in_camera_pose*world_point;
      if (camera_point.z()<=0 || camera_point.z()>_z_far || camera_point.z()<_z_near)
        return false;
      Eigen::Vector3f projected_point=_camera_matrix*camera_point;
      image_point=projected_point.head<2>()*(1./projected_point.z());
      if(image_point.x()<0 || image_point.x()>_cols-1)
        return false;
      if(image_point.y()<0 || image_point.y()>_rows-1)
        return false;
      return true;
    }

    //! projects a bunch of world points on the image
    //! @param image_points: the points on the image
    //! @param world points: the input world points
    //! @param keep_indices: if true, image_points has the same size of world points
    //! Invalid points are marked with (-1, -1). If false only the points in the set are returned
    //! @returns the number of points that fall inside the image
    int projectPoints(Vector2fVector& image_points,
                      const Vector3fVector& world_points,
                      bool keep_indices=false);

    inline const Eigen::Isometry3f& worldInCameraPose() const {return _world_in_camera_pose;}
    inline void setWorldInCameraPose(const Eigen::Isometry3f& pose)  {_world_in_camera_pose=pose;}
    inline const Eigen::Matrix3f& cameraMatrix() const {return _camera_matrix;}
    inline const int rows() const {return _rows;}
    inline const int cols() const {return _cols;}

    
  protected:
    int _rows; // image_size
    int _cols; //
    int _z_near;
    int _z_far; 
    Eigen::Matrix3f _camera_matrix;
    Eigen::Isometry3f _world_in_camera_pose;
  };
