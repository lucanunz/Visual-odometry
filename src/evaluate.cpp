#include <iostream>
#include "utils.h"
#include "files_utils.h"

Vector3fVector read(const std::string& file_path){
    std::ifstream input_stream(file_path);
    std::string word;
    std::string line;
    Vector3fVector points;
    float n=0.f;
    if(!input_stream.is_open()){
        std::cout << "Unable to open " << file_path << std::endl;
        return points;
    }
    while (std::getline(input_stream, line)) {
        std::stringstream ss(line);
        Eigen::Vector3f point;
        if(line.empty())
            continue;
        for (int i = 0; i < 3; i++) {
            ss >> n;
            point(i)=n;
        }
        points.push_back(point);
    }
    input_stream.close();
    return points;
}
 VectorIsometry get_gt_data(const std::string& file_path){
    std::ifstream input_stream(file_path);
    std::string word;
    std::string line;
    VectorIsometry data;
    float n=0.f;
    if(!input_stream.is_open()){
        std::cout << "Unable to open " << file_path << std::endl;
        return data;
    }
    while (std::getline(input_stream, line)) {
        std::stringstream ss(line);
        Eigen::Vector3f info;
        if(line.empty())
            continue;
        for (int i=0;i<4;i++)
            ss >> word;
        for (int i = 0; i < 3; i++) {
            ss >> n;
            info(i)=n;
        }
        Eigen::Isometry3f X=Eigen::Isometry3f::Identity();
        X.linear()=RotationZ(info(2));
        X.translation() << info.head<2>(),0.f;
        data.push_back(X);
    }
    input_stream.close();
    return data;
}
VectorIsometry get_est_data(const std::string& file_path){
    std::ifstream input_stream(file_path);
    std::string word;
    std::string line;
    VectorIsometry trajectory;
    float n=0.f;
    if(!input_stream.is_open()){
        std::cout << "Unable to open " << file_path << std::endl;
        return trajectory;
    }
    while (std::getline(input_stream, line)) {
        std::stringstream ss(line);
        Eigen::Vector3f point;
        if(line.empty())
            continue;
        for (int i = 0; i < 3; i++) {
            ss >> n;
            point(i)=n;
        }
        Eigen::Matrix3f rot=Eigen::Matrix3f::Identity();
        for(int i=0;i<3;i++){
            std::getline(input_stream, line);
            std::stringstream sss(line);
            for(int j=0;j<3;j++){
                sss>>n;
                rot(i,j)=n;
            }
        }
        Eigen::Isometry3f X=Eigen::Isometry3f::Identity(); X.linear()=rot; X.translation()=point;
        trajectory.push_back(X);
    }
    input_stream.close();
    return trajectory;
}
int main(){
    // Vector3fVector gt=read("trajectory_gt.txt");
    // Vector3fVector est=read("trajectory_est_complete.txt");
    // for(size_t i=0;i<gt.size();i++){
    //     std::cout << gt[i].x()/est[i].x() << ", " << gt[i].y()/est[i].y() << ", " << gt[i].z()/est[i].z() << std::endl;
    //     std::cout << gt[i].norm()/est[i].norm() << std::endl;
    // }
    const std::string path="/home/luca/vo_data/data/";
    VectorIsometry gt_data=get_gt_data(path+"trajectory.dat");
    VectorIsometry traj_est_data=get_est_data("trajectory_est_data.txt");
    std::vector<float> orientation_error;
    std::vector<float> ratio;
    for(size_t i=1;i<gt_data.size();i++){
        Eigen::Isometry3f X_prev=traj_est_data[i-1];
        Eigen::Isometry3f X_curr=traj_est_data[i];
        Eigen::Isometry3f X_prev_gt=gt_data[i-1];
        Eigen::Isometry3f X_curr_gt=gt_data[i];
        auto X_rel=X_prev.inverse()*X_curr;
        auto X_rel_gt=X_prev_gt.inverse()*X_curr_gt;
        Eigen::Matrix3f error=Eigen::Matrix3f::Identity()-X_rel.linear().transpose()*X_rel_gt.linear();
        orientation_error.push_back(error.trace());
        ratio.push_back(X_rel_gt.translation().norm()/X_rel.translation().norm());
    }
    std::ofstream output_file("out_performance.txt");
    for(size_t i=1;i<ratio.size();i++){
        std::cout << orientation_error[i] << ", " << ratio[i] << std::endl;
        output_file << orientation_error[i] << " " << ratio[i] << std::endl;
    }
    output_file.close();
    
}