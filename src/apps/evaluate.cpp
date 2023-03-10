#include <iostream>
#include <cmath>
#include "utils.h"
#include "files_utils.h"
#include "evaluation_utils.h"

int main(int argc, char* argv[]) {
    
    if(argc < 2){
        std::cout << "Error: need path parameter to read data" << std::endl;
        return -1;
    }

    std::string path(argv[1]);
    if(path.back() != '/')
        path.push_back('/');

    const IsometryVector gt_data=get_gt_data(path+"trajectory.dat");
    const IsometryVector traj_est_data=get_est_data("trajectory_est_data.txt");
    std::vector<float> orientation_error;
    std::vector<float> ratio;

    std::ofstream output_file("out_performance.txt");
    for(size_t i=1;i<gt_data.size();i++){
        const Eigen::Isometry3f X_prev=traj_est_data[i-1];
        const Eigen::Isometry3f X_curr=traj_est_data[i];

        const Eigen::Isometry3f X_prev_gt=gt_data[i-1];
        const Eigen::Isometry3f X_curr_gt=gt_data[i];

        const auto X_rel=X_prev.inverse()*X_curr;
        const auto X_rel_gt=X_prev_gt.inverse()*X_curr_gt;

        const Eigen::Matrix3f error=Eigen::Matrix3f::Identity()-X_rel.linear().transpose()*X_rel_gt.linear();

        orientation_error.push_back(error.trace());
        ratio.push_back(X_rel.translation().norm()/X_rel_gt.translation().norm());
        output_file << orientation_error.back() << " " << ratio.back() << std::endl;
    }        
    
    output_file.close();

    const float median_ratio=1./median(ratio);
    std::cout << "ratio used for map correction: " << median_ratio << std::endl;

    float position_rmse=0.f;
    for(size_t i=0;i<gt_data.size();i++)
        position_rmse+=pow((gt_data[i].translation() - traj_est_data[i].translation()*median_ratio).norm(),2);

    position_rmse*=(1.f/gt_data.size());
    position_rmse=sqrt(position_rmse);
    
    std::cout << "RMSE position: " << position_rmse << std::endl;

    const Vector3fVector map_est=read_eigen_vectors<3>("map.txt");
    Vector3fVector map_corrected;

    for(const auto& p:map_est)
        map_corrected.push_back(p*median_ratio);
    
    write_eigen_vectors_to_file("map_corrected.txt",map_corrected);

    const Vector10fVector map_appearances=read_eigen_vectors<10>("map_appearances.txt");
    Vector3fVector world_points;
    Vector10fVector world_points_appearances;
    if(!get_meas_content(path+"world.dat",world_points_appearances,world_points,true)){
        std::cout << "Unable to open world file\n";
        return -1;     
    }
    
    Vector3fVector world_pruned;
    Vector6fVector world_map_points;
    float rmse=0.f;
    for(size_t i=0;i<map_corrected.size();i++){
        for(size_t j=0;j<world_points.size();j++)
            if(map_appearances[i]==world_points_appearances[j]){
                world_map_points.push_back((Vector6f() << map_corrected[i],world_points[j]).finished());
                rmse+=pow((map_corrected[i]-world_points[j]).norm(),2);
                world_pruned.push_back(world_points[j]);
                break;
            }
    }
    rmse*=(1.f/world_pruned.size());
    rmse=sqrt(rmse);

    std::cout << "RMSE map: " << rmse << std::endl;
    write_eigen_vectors_to_file("arrows.txt",world_map_points);
    write_eigen_vectors_to_file("world_pruned.txt",world_pruned);

}