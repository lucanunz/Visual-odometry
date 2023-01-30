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

int main() {
    //Testing with synthetic data
    
    Eigen::Isometry3f X_gt;
    generate_isometry3f(X_gt);

    Vector3fVector world_points, world_points_transformed;
    
    generate_points3d(X_gt,90,world_points,world_points_transformed); //we choose to ignore the second argument
    write_eigen_vectors_to_file("world_points.txt",world_points);
    write_eigen_vectors_to_file("world_points_transformed.txt",world_points_transformed);

    Eigen::Matrix3f k;
    k << 150.f,0.f,320.f,
        0.f,150.f,240.f,
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

    Vector6f disturbance;
    disturbance << 0.1f,0.2f,0.3f,0.1f,0.1f,0.2f;
    cam.setWorldInCameraPose(X_gt*v2tEuler(disturbance));

    PICPSolver solver;
    solver.setKernelThreshold(10000);
    solver.init(cam,world_points,current_measurements);
    for(int i=0;i<30;i++)
        solver.oneRound(correspondences,false);

    cam=solver.camera();
    std::cout << "R estimated:\n";
    std::cout << cam.worldInCameraPose().linear() << std::endl;
    std::cout << "t estimated: " << cam.worldInCameraPose().translation().transpose() << std::endl;
    std::cout << "R gt:\n";
    std::cout << X_gt.linear() << std::endl;
    std::cout << "t gt: " << X_gt.translation().transpose() << std::endl;
    return 0;
}