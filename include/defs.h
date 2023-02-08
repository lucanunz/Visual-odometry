#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include <iostream>
typedef Eigen::Matrix<float, 11, 1> Vector11f;
typedef Eigen::Matrix<float, 10, 1> Vector10f;
typedef Eigen::Matrix<float, 9, 1> Vector9f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 1, 9> RowVector9f;
typedef Eigen::Matrix<float, 9, 9> Matrix9f;
typedef Eigen::Matrix<float, 2, 3> Matrix2_3f;
typedef Eigen::Matrix<float, 2, 6> Matrix2_6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 3, 6> Matrix3_6f;

typedef std::pair<Eigen::Isometry3f,Eigen::Isometry3f> IsometryPair;
typedef std::pair<int,int> IntPair;
typedef std::vector<IntPair > IntPairVector;

typedef std::vector<Vector11f, Eigen::aligned_allocator<Vector11f> > Vector11fVector;
typedef std::vector<Vector6f, Eigen::aligned_allocator<Vector6f> > Vector6fVector;
typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > Vector4fVector;
typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
typedef std::vector<Vector10f, Eigen::aligned_allocator<Vector10f> > Vector10fVector;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > Vector2iVector;
typedef std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > VectorIsometry;

//! class to represent a single point of a PointCloud. Each object is characterized by its coordinates
//! and its appearance. In this project we use 2D pointclouds for the measurements, and 3D for the world points.
template <int dim>
class PointCloud {
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