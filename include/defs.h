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
typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > Vector4fVector;
typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
typedef std::vector<Vector10f, Eigen::aligned_allocator<Vector10f> > Vector10fVector;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > Vector2iVector;
typedef std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > VectorIsometry;

template <int dim>
class PointCloud {
    public:
    PointCloud(){}
    PointCloud(const Eigen::Matrix<float,dim,1>& point, const Vector10f& appearance) :
        _point(point), _appearance(appearance) {}

    inline Eigen::Matrix<float,dim,1> point() const { return _point;   }
    inline Vector10f appearance() const { return _appearance; }

    protected:
    Eigen::Matrix<float,dim,1> _point;
    Vector10f _appearance;
};

template <int dim>
class PointCloudVector{
    typedef std::vector<Eigen::Matrix<float,dim,1>, Eigen::aligned_allocator<Eigen::Matrix<float,dim,1> > > PointsVec;
    public:
    PointCloudVector(){}
    PointCloudVector(size_t N): _vector(N) {}
    PointCloudVector& operator=(PointCloudVector other){
        std::swap(_vector,other.vector());
        return *this;
    }
    inline void push_back(const PointCloud<dim>& pc){ _vector.push_back(pc); }
    inline PointCloud<dim> back() const { return _vector.back(); }
    inline void resize(size_t N){ _vector.resize(N);}
    inline size_t size() const {return _vector.size(); }
    inline void clear(){ _vector.clear(); }
    
    PointCloud<dim> operator [](int i) const {return _vector.at(i); }
    PointCloud<dim>& operator [](int i) {return _vector.at(i); }

    PointsVec points(){
        PointsVec ret(_vector.size());
        if(_vector.size()>0){
            for(size_t i=0;i<_vector.size();i++)
                ret[i]=_vector[i].point();
        }
        return ret;
    }
    Vector10fVector appearances(){
        Vector10fVector ret(_vector.size());
        if(_vector.size()>0){
            for(size_t i=0;i<_vector.size();i++)
                ret[i]=_vector[i].appearance();
        }
        return ret;
    }
    std::vector<PointCloud<dim>>& vector(){ return _vector; }
    protected:
    std::vector<PointCloud<dim>> _vector;
};