#include "utils.h"
double getTime(){
    struct timeval tv;
    gettimeofday(&tv, 0);
    return 1e3*tv.tv_sec+tv.tv_usec*1e-3;
}

void generate_isometry3f(Eigen::Isometry3f& X){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-1.0f, 1.0f);

    Eigen::Vector3f a = Eigen::Vector3f::NullaryExpr(3,1,[&](){return dis(gen);});
    a.normalize();
    Eigen::AngleAxisf random_angle_axis(dis(gen),a);
    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f(random_angle_axis);

    X.linear()=rotation_matrix;
    X.translation()=Eigen::Vector3f::NullaryExpr(3,1,[&](){return dis(gen);});
}

Vector3fVector generate_points3d(const int& num_points){
    //generating data in the same range of the ones provided
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-10.f, 10.0f);
    Vector3fVector points; points.resize(num_points);
    for(int i=0;i<num_points;i++){
        Eigen::Vector3f p;
        p << dis(gen),dis(gen),dis(gen)*0.1f+1.0f;
        points[i]=p;
    }
    return points;
}

bool triangulate_point(const Eigen::Vector3f& d1,const Eigen::Vector3f& d2,const Eigen::Vector3f& p2,Eigen::Vector3f& p){
    Eigen::Matrix<float,3,2> D;
    D.col(0)=-d1;
    D.col(1)=d2;
    const Eigen::Vector2f ss=-(D.transpose() * D).ldlt().solve(D.transpose() * p2);
    if(ss(0)<0 || ss(1)<0)
        return false;

    const Eigen::Vector3f p1_triangulated=ss(0)*d1;
    const Eigen::Vector3f p2_triangulated=p2+ss(1)*d2;
    //std::cout << "error: " << (p1_triangulated-p2_triangulated).norm() << std::endl;
    p=0.5f*(p1_triangulated+p2_triangulated);
    return true;
}

int triangulate_points(const Eigen::Matrix3f& k, const Eigen::Isometry3f& X, const IntPairVector& correspondences,
                        const Vector2fVector& p1_img, const Vector2fVector& p2_img, Vector3fVector& triangulated){
    const Eigen::Isometry3f iX = X.inverse();
    const Eigen::Matrix3f iK = k.inverse();
    const Eigen::Matrix3f iRiK = iX.linear()*iK;
    const Eigen::Vector3f t= iX.translation();
    int n_success=0;
    triangulated.resize(correspondences.size());
    for (const IntPair& correspondence: correspondences){
        const int idx_first=correspondence.first;
        const int idx_second=correspondence.second;
        Eigen::Vector3f d1;
        d1 << p1_img[idx_first],1;
        d1 = iK*d1;
        Eigen::Vector3f d2;
        d2 << p2_img[idx_second],1;
        d2 = iRiK*d2;
        Eigen::Vector3f p;
        if(triangulate_point(d1,d2,t,p)){
            triangulated[n_success]=p;
            n_success++;
        }
    }
    triangulated.resize(n_success);
    return n_success;
}
int triangulate_points(const Eigen::Matrix3f& k, const Eigen::Isometry3f& X, const IntPairVector& correspondences,
                        const Vector2fVector& p1_img, const Vector2fVector& p2_img, Vector3fVector& triangulated, IntPairVector& correspondences_new){
    const Eigen::Isometry3f iX = X.inverse();
    const Eigen::Matrix3f iK = k.inverse();
    const Eigen::Matrix3f iRiK = iX.linear()*iK;
    const Eigen::Vector3f t= iX.translation();
    int n_success=0;
    triangulated.resize(correspondences.size());
    correspondences_new.resize(correspondences.size());
    for (const IntPair& correspondence: correspondences){
        const int idx_first=correspondence.first;
        const int idx_second=correspondence.second;
        Eigen::Vector3f d1;
        d1 << p1_img[idx_first],1;
        d1 = iK*d1;
        Eigen::Vector3f d2;
        d2 << p2_img[idx_second],1;
        d2 = iRiK*d2;
        Eigen::Vector3f p;
        if(triangulate_point(d1,d2,t,p)){
            correspondences_new[n_success]=IntPair(idx_second,n_success);
            triangulated[n_success]=p;
            n_success++;
        }
    } 
    triangulated.resize(n_success);
    correspondences_new.resize(n_success);
    return n_success;
}
int triangulate_points(const Eigen::Matrix3f& k, const Eigen::Isometry3f& X, const IntPairVector& correspondences,
                        const PointCloudVector<2>& pc_1, const PointCloudVector<2>& pc_2, PointCloudVector<3>& triangulated, IntPairVector& correspondences_new){
    const Eigen::Isometry3f iX = X.inverse();
    const Eigen::Matrix3f iK = k.inverse();
    const Eigen::Matrix3f iRiK = iX.linear()*iK;
    const Eigen::Vector3f t= iX.translation();
    int n_success=0;
    triangulated.clear(); triangulated.reserve(correspondences.size());
    correspondences_new.resize(correspondences.size());
    for (const IntPair& correspondence: correspondences){
        const int idx_first=correspondence.first;
        const int idx_second=correspondence.second;
        Eigen::Vector3f d1;
        d1 << pc_1.points().at(idx_first),1;
        d1 = iK*d1;
        Eigen::Vector3f d2;
        d2 << pc_2.points().at(idx_second),1;
        d2 = iRiK*d2;
        Eigen::Vector3f p;
        if(triangulate_point(d1,d2,t,p)){
            correspondences_new[n_success]=IntPair(idx_second,n_success);
            triangulated.push_back(PointCloud<3>(p,pc_2.appearances().at(idx_second)));
            n_success++;
        }
    } 
    triangulated.resize(n_success);
    correspondences_new.resize(n_success);
    return n_success;
}
