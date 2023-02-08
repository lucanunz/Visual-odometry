#include <iostream>
#include <cmath>
#include "utils.h"
#include "files_utils.h"

template <int dim>
std::vector<Eigen::Matrix<float,dim,1>,Eigen::aligned_allocator<Eigen::Matrix<float,dim,1>> > read(const std::string& file_path){
    std::ifstream input_stream(file_path);
    std::string word;
    std::string line;
    std::vector<Eigen::Matrix<float,dim,1>,Eigen::aligned_allocator<Eigen::Matrix<float,dim,1>> > points;
    float n=0.f;
    if(!input_stream.is_open()){
        std::cout << "Unable to open " << file_path << std::endl;
        return points;
    }
    while (std::getline(input_stream, line)) {
        std::stringstream ss(line);
        Eigen::Matrix<float,dim,1> point;
        if(line.empty())
            continue;
        for (int i = 0; i < dim; i++) {
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
float median(std::vector<float> &v)
{
    size_t n = v.size() / 2;
    nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
}
int main(){
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
    for(size_t i=1;i<ratio.size();i++)
        output_file << orientation_error[i] << " " << ratio[i] << std::endl;
    
    output_file.close();
    Vector3fVector map_est=read<3>("map.txt");
    Vector3fVector map_corrected;
    float mean_ratio=median(ratio);
    std::cout << mean_ratio << std::endl;
    for(const auto& p:map_est)
        map_corrected.push_back(p*mean_ratio);
    
    write_eigen_vectors_to_file("map_corrected.txt",map_corrected);
    Vector10fVector map_appearances=read<10>("map_appearances.txt");
    Vector3fVector world_points;
    Vector10fVector world_points_appearances;
    if(!get_meas_content(path+"world.dat",world_points_appearances,world_points,true)){
        std::cout << "Unable to open world file\n";
        return -1;     
    }
    Vector3fVector world_pruned;
    Vector6fVector world_map_points;
    float rms=0.f;
    for(size_t i=0;i<map_corrected.size();i++){
        for(size_t j=0;j<world_points.size();j++)
            if(map_appearances[i]==world_points_appearances[j]){
                world_map_points.push_back((Vector6f() << map_corrected[i],world_points[j]).finished());
                rms+=pow((map_corrected[i]-world_points[j]).norm(),2);
                world_pruned.push_back(world_points[j]);
                break;
            }
    }
    rms*=(1.f/world_pruned.size());
    rms=sqrt(rms);
    std::cout << "RMS: " << rms << std::endl;
    write_eigen_vectors_to_file("arrows.txt",world_map_points);
    write_eigen_vectors_to_file("world_pruned.txt",world_pruned);

}