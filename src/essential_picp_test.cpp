// In this file we first generate synthetic data and 3 sets of measurements, then perform the estimation of
// the relative pose between two camera using epipolar geometry. Given this transformation, we triangulate
// to estimate the position of the points in the world, and then use this estimate to perform picp
#include <iostream>
#include "utils.h"
#include "files_utils.h"
#include "camera.h"
#include "picp_solver.h"
#include "epipolar_utils.h"

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
    std::cout << "t ratio: ";
    for (int i=0;i<3;i++)
        std::cout << t_est(i)/t_gt(i) << ", ";
    std::cout << std::endl;
}
int main() {
    //Testing with synthetic data
    
    Eigen::Isometry3f X_gt1; //pose of the world(camera position 0) in the first camera
    generate_isometry3f(X_gt1);

    Vector3fVector world_points_gt=generate_points3d(1000);
    write_eigen_vectors_to_file("world_points_gt.txt",world_points_gt);

    Eigen::Matrix3f k;
    k << 150.f,0.f,320.f,
        0.f,150.f,240.f,
        0.f,0.f,1.f;
    Camera cam(480,640,0,10,k);
    
    Vector2fVector reference_image_points;
    Vector2fVector current_measurements;

    //since we keep indices, the i-th proj is the i-th world point.
    cam.projectPoints(reference_image_points,world_points_gt,true);
    cam.setWorldInCameraPose(X_gt1);
    cam.projectPoints(current_measurements,world_points_gt,true);


    IntPairVector correspondences;
    computeFakeCorrespondences(correspondences, reference_image_points, current_measurements);

    //Estimate of X_gt1
    const Eigen::Isometry3f X_est = estimate_transform(cam.cameraMatrix(), correspondences,reference_image_points,current_measurements);

    print_comparison(X_est,X_gt1,"**********EPIPOLAR RESULTS**********");

    Vector3fVector world_points_est;
    IntPairVector correspondences_new;
    triangulate_points(k,X_est,correspondences,reference_image_points,
                        current_measurements,world_points_est,correspondences_new);

    write_eigen_vectors_to_file("world_triang.txt",world_points_est);

    Eigen::Isometry3f X_gt2;
    generate_isometry3f(X_gt2); //  pose of the world in camera 2

    cam.setWorldInCameraPose(X_gt2);
    cam.projectPoints(current_measurements,world_points_gt,true);
    
    PICPSolver solver;
    solver.setKernelThreshold(10000);

    Vector3fVector points_in_cameraframe1;
    for(const auto& p : world_points_est)
        points_in_cameraframe1.push_back(X_est*p);

    cam.setWorldInCameraPose(Eigen::Isometry3f::Identity());
    solver.init(cam,points_in_cameraframe1,current_measurements);
    for(int i=0;i<100;i++)
        solver.oneRound(correspondences_new,false);
    
    cam=solver.camera();// this should estimate the pose of the first camera in the frame of the second
    print_comparison(cam.worldInCameraPose(),X_gt2*(X_gt1.inverse()),"**********PICP RESULTS**********");
    //std::cout << (Eigen::Matrix3f::Identity()-(cam.worldInCameraPose().inverse()*(X_gt2*(X_gt1.inverse()))).linear()).trace() << std::endl;
    
    return 0;
}