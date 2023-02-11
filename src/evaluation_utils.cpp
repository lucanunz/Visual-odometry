#include "evaluation_utils.h"

IsometryVector get_gt_data(const std::string& file_path){
    std::ifstream input_stream(file_path);
    std::string word;
    std::string line; 
    IsometryVector data;
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
IsometryVector get_est_data(const std::string& file_path){
    std::ifstream input_stream(file_path);
    std::string word;
    std::string line;
    IsometryVector trajectory;
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
            std::stringstream ss_in(line);
            for(int j=0;j<3;j++){
                ss_in >> rot(i,j);
            }
        }
        Eigen::Isometry3f X=Eigen::Isometry3f::Identity(); X.linear()=rot; X.translation()=point;
        trajectory.push_back(X);
    }
    input_stream.close();
    return trajectory;
}
float median(std::vector<float> v)
{
    const size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
}