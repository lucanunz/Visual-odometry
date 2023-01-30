#include <iostream>
#include "utils.h"
#include "files_utils.h"
#include "camera.h"

void computeFakeCorrespondences(IntPairVector& correspondences,
				const Vector2fVector reference_image_points,
				const Vector2fVector current_image_points){
    correspondences.resize(current_image_points.size());
    int num_correspondences=0;
    assert(reference_image_points.size()==current_image_points.size());
  
    for (size_t i=0; i<reference_image_points.size(); i++){
        const Eigen::Vector2f& reference_point=reference_image_points[i];
        const Eigen::Vector2f& current_point=current_image_points[i];
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
    //Testing with synthetic data------------------------------------------------------------------------------start
    
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
    Camera cam2(480,640,0,10,k,X_gt);
    
    Vector2fVector reference_image_points;
    Vector2fVector current_image_points;

    //since we keep indices, the i-th proj is the i-th world point.
    cam.projectPoints(reference_image_points,world_points,true);
    cam2.projectPoints(current_image_points,world_points,true);

    //Now we have generated the synthetic measurements reference_image_points and current_image_points
    
    IntPairVector correspondences;
    computeFakeCorrespondences(correspondences, reference_image_points, current_image_points);

    const Eigen::Matrix3f E_est=estimate_essential(k,correspondences,reference_image_points,current_image_points);
    
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(E_est);

    // const Eigen::Matrix3f E_gt = transform2essential(X_gt);
    // std::cout << "E_gt:\n" << E_gt << std::endl << std::endl;
    // std::cout << "E_est:\n" << E_est << std::endl << std::endl;
    // std::cout << "Ratio of the essentials:\n";
    // for (int i=0;i<3;i++){
    //     for(int j=0;j<3;j++)
    //         std::cout << E_est(i,j)/E_gt(i,j) << " ";
    //     std::cout << std::endl;
    // }
    // std::cout << std::endl;

    const Eigen::Isometry3f X_est = estimate_transform(k, correspondences,reference_image_points,current_image_points);
    std::cout << "R estimated:\n" << X_est.linear() << std::endl;
    std::cout << "R gt:\n" << X_gt.linear() << std::endl;
    std::cout << "t ratio:\n" << std::endl;
    const Eigen::Vector3f t_est=X_est.translation();
    const Eigen::Vector3f t_gt=X_gt.translation();
    for (int i=0;i<3;i++)
        std::cout << t_est(i)/t_gt(i) << std::endl;
    
    Vector3fVector triangulated_world_points;
    triangulate_points(k,X_est,correspondences,reference_image_points,current_image_points,triangulated_world_points);

    write_eigen_vectors_to_file("p_triang.txt",triangulated_world_points);
    //Testing with synthetic data------------------------------------------------------------------------------end
    return 0;
}