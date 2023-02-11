#pragma once
#include "defs.h"
#include <fstream>
#include <utils.h>

template <int dim>
std::vector<Eigen::Matrix<float,dim,1>,Eigen::aligned_allocator<Eigen::Matrix<float,dim,1>> > read_eigen_vectors(const std::string& file_path){
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

float median(std::vector<float> v);

IsometryVector get_gt_data(const std::string& file_path);

IsometryVector get_est_data(const std::string& file_path);