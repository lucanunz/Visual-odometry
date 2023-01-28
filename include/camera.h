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
           const Eigen::Matrix3f& camera_matrix=Eigen::Matrix3f::Identity());

    // ! projects a single point on the image plane
    // ! @returns false if the point is behind the camera or outside the plane
    inline  bool projectPoint(Eigen::Vector2f& image_point,
                              const Eigen::Vector3f& world_point){
      if (world_point.z()<=0 || world_point.z()>_z_far || world_point.z()<_z_near)
        return false;
      Eigen::Vector3f projected_point=_camera_matrix*world_point;
      image_point=projected_point.head<2>()*(1./projected_point.z());
      if(image_point.x()<0 || image_point.x()>_cols-1)
        return false;
      if(image_point.y()<0 || image_point.y()>_rows-1)
        return false;
      return true;
    }

    //! projects a bunch of world points on the image
    //! @param image_points: the points on the image with the associated id
    //! @param world points: the input world points. The id of the i-th world point is i.
    //! @returns the number of points that fall inside the image
    int projectPoints(Vector3fVector& image_points,
                      const Vector3fVector& world_points);
  
    inline const Eigen::Matrix3f& cameraMatrix() const {return _camera_matrix;}
    inline const int rows() const {return _rows;}
    inline const int cols() const {return _cols;}

    
  protected:
    int _rows; // image_size
    int _cols; //
    int _z_near;
    int _z_far; 
    Eigen::Matrix3f _camera_matrix;
  };
