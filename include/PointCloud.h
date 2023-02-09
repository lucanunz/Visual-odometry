#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include "defs.h"
//! class to represent a single point of a PointCloud. Each object is characterized by its coordinates
//! and its appearance. In this project we use 2D pointclouds for the measurements, and 3D for the world points.
template <int dim>
class PointCloud{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PointCloud(){}
    PointCloud(const Eigen::Matrix<float,dim,1>& point, const Vector10f& appearance) :
        _point(point), _appearance(appearance) {}

    inline Eigen::Matrix<float,dim,1> point() const { return _point;   }
    inline Vector10f appearance() const { return _appearance; }

    protected:
    Eigen::Matrix<float,dim,1> _point;
    Vector10f _appearance;
};

//! class that represents a vector of point clouds
template <int dim>
class PointCloudVector{
    typedef std::vector<Eigen::Matrix<float,dim,1>,Eigen::aligned_allocator<Eigen::Matrix<float,dim,1>> > PointsVec;
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PointCloudVector(){}
    PointCloudVector(size_t N): _points(N),_appearances(N) {}

    void push_back(const PointCloud<dim>& pc){ 
        _points.push_back(pc.point());
        _appearances.push_back(pc.appearance());
    }

    void resize(size_t N){ 
        _points.resize(N); 
        _appearances.resize(N); 
    }
    void reserve(size_t N){ 
        _points.reserve(N); 
        _appearances.reserve(N); 
    }
    inline size_t size() const {return _points.size(); }
    void clear(){ 
        _points.clear(); 
        _appearances.clear(); 
    }
    void update(const PointCloudVector<dim>& cloud){
        bool found=false;
        for(size_t i=0;i<cloud.size();i++){
            for(size_t j=0;j<_appearances.size() && !found ;j++){
                if (_appearances[j]==cloud.appearances().at(i)){
                    _points[j]=cloud.points().at(i);
                    found=true;
                }
            }
            if(found)
                found=false;
            else
                this->push_back(PointCloud<dim>(cloud.points().at(i),cloud.appearances().at(i)));
        }
    }

    inline PointsVec& points(){ return _points;}
    inline Vector10fVector& appearances(){ return _appearances;}
    inline PointsVec points() const { return _points;}
    inline Vector10fVector appearances() const { return _appearances;}
    protected:
    PointsVec _points;
    Vector10fVector _appearances;
};

inline PointCloudVector<3> operator*(Eigen::Isometry3f X, const PointCloudVector<3> pc){
    PointCloudVector<3> ret;
    for(size_t i=0;i<pc.size();i++)
        ret.push_back(PointCloud<3>(X*pc.points().at(i),pc.appearances().at(i)));
    return ret;
}