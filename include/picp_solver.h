#pragma once
#include "defs.h"
#include "camera.h"
#include "utils.h"
  /**
     Solver for point-to-camera problem.
     Implements a least squares solver using translation+euler angles as minimal parameterization
     A simple saturating robust kernel, and an adjustable damping coefficient;
     To use it:
     - create an object
     - initialize it passing:
       - the image points (that represent the measurements)
       - the world points (that represent the model)
       - A camera whose pose is initialized at initial_guess
     - call oneRound(<correspondences>) a bunch of times, with the correspondences returned by the finder;
       at each call, the solution will be subject to one ls operation
   */
  class PICPSolver{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    //! ctor
    PICPSolver();

    //! init method, call it at the beginning
    //! @param camera: the camera
    //! @param world_points: the points of the world
    //! @param image_points: the points of the reference
    void init(const Camera& camera,
	      const Vector3fVector& world_points,
	      const Vector2fVector& image_points);
  
    inline float kernelThreshold() const {return _kernel_thereshold;}

    inline void setKernelThreshold(float kernel_threshold) 
    {_kernel_thereshold=kernel_threshold;}


  
    //! accessor to the camera
    const Camera& camera() const {return _camera;}

    //! chi square of the "good" points
    const float chiInliers() const {return _chi_inliers;}
    
    //! chi square of the "bad" points
    const float chiOutliers() const {return _chi_outliers;}
    
    //! number of inliers (an inlier is a point whose error is below kernel threshold)
    const int numInliers() const {return _num_inliers;}
    
    //! performs one iteration of optimization
    //! @param correspondences: the correspondences (first: measurement, second:model);
    //! param keep_outliers: if true, the outliers are considered in the optimization 
    //! (but cut by the kernel)
    bool oneRound(const IntPairVector& correspondences, bool keep_outliers);

  protected:

    bool errorAndJacobian(Eigen::Vector2f& error,
			  Matrix2_6f& jacobian,
			  const Eigen::Vector3f& world_point,
			  const Eigen::Vector2f& reference_image_point);

    void linearize(const IntPairVector& correspondences, bool keep_outliers);

			       
    Camera _camera;                  //< this will hold our state
    float _kernel_thereshold;        //< threshold for the kernel
    float _damping;                  //< damping, to slow the solution
    int _min_num_inliers;            //< if less inliers than this value, the solver stops
    const Vector3fVector* _world_points;
    const Vector2fVector* _reference_image_points;
    Matrix6f _H;
    Vector6f _b;
    float _chi_inliers;
    float _chi_outliers;
    int _num_inliers;
  };
