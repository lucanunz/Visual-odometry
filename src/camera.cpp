#include "camera.h"

Camera::Camera(int rows,
                int cols,
                int z_near,
                int z_far,
                const Eigen::Matrix3f& camera_matrix):
    _rows(rows),
    _cols(cols),
    _z_near(z_near),
    _z_far(z_far),
    _camera_matrix(camera_matrix){}


  int Camera::projectPoints(Vector3fVector& image_points,
                            const Vector3fVector& world_points){
    image_points.resize(world_points.size());
    int num_image_points=0;
    const Eigen::Vector2f point_outside(-1,-1);

    for(size_t i=0; i<world_points.size(); i++){
      const Eigen::Vector3f world_point=world_points[i];
      Eigen::Vector2f image_point_coords;
      bool is_inside=projectPoint(image_point_coords,world_point);

      if (is_inside){
        image_points[num_image_points] << i,image_point_coords;
        num_image_points++;
      }
    }
    image_points.resize(num_image_points);
    return num_image_points;
  }