// This file generate synthetic data and measurements from 2 poses and estimates the relative pose using picp
#include <iostream>
#include "utils.h"
#include "files_utils.h"
#include "camera.h"
#include "picp_solver.h"

void computeFakeCorrespondences(IntPairVector& correspondences,
				const Vector2fVector reference_image_points,
				const Vector2fVector current_measurements){
    correspondences.resize(current_measurements.size());
    int num_correspondences=0;
    assert(reference_image_points.size()==current_measurements.size());
  
    for (size_t i=0; i<reference_image_points.size(); i++){
        const Eigen::Vector2f& reference_point=reference_image_points[i];
        const Eigen::Vector2f& current_point=current_measurements[i];
        IntPair& correspondence=correspondences[num_correspondences];
        if (reference_point.x()<0 || current_point.x()<0) //if one of them is the invalid point
            continue;
        correspondence.first=i;
        correspondence.second=i;
        num_correspondences++;
    }
    correspondences.resize(num_correspondences);
}

void print_comparison(const Eigen::Isometry3f& X_est, const Eigen::Isometry3f& X_gt,const std::string title={}){
    if(!title.empty())
        std::cout << title << std::endl;
    std::cout <<"R estimated:\n";
    std::cout << X_est.linear() << std::endl;
    std::cout << "R gt:\n";
    std::cout << X_gt.linear() << std::endl;
    const Eigen::Vector3f t_est=X_est.translation();
    const Eigen::Vector3f t_gt=X_gt.translation();
    std::cout << "t_est: " << t_est.transpose();
    std::cout << "\nt_gt: " << t_gt.transpose();
    std::cout << std::endl;
}

int main() {
    //Testing with synthetic data
    
    Eigen::Isometry3f X_gt;
    generate_isometry3f(X_gt);

    Vector3fVector world_points=generate_points3d(1000);

    Eigen::Matrix3f k;
    k << 180.f,0.f,320.f,
        0.f,180.f,240.f,
        0.f,0.f,1.f;
    Camera cam(480,640,0,10,k);
    
    Vector2fVector reference_image_points;
    Vector2fVector current_measurements;

    //since we keep indices, the i-th proj is the i-th world point.
    cam.projectPoints(reference_image_points,world_points,true);
    cam.setWorldInCameraPose(X_gt);
    cam.projectPoints(current_measurements,world_points,true);

    //Now we have generated the synthetic measurements reference_image_points and current_measurements

    IntPairVector correspondences;
    computeFakeCorrespondences(correspondences, reference_image_points, current_measurements);

    cam.setWorldInCameraPose(Eigen::Isometry3f::Identity());

    PICPSolver solver;
    solver.setKernelThreshold(10000);
    solver.init(cam,world_points,current_measurements);
    for(int i=0;i<1000;i++)
        solver.oneRound(correspondences,false);
    
    cam=solver.camera();
    print_comparison(cam.worldInCameraPose(),X_gt,"PICP solver");
    return 0;
}