// This file tests if data files are read correctly
#include <iostream>
#include "utils.h"
#include "files_utils.h"
#include "camera.h"

int main(int argc, char* argv[]) {
    if(argc < 2){
        std::cout << "Error: need path parameter to read data" << std::endl;
        return -1;
    }

    std::string path(argv[1]);
    if(path.back() != '/')
        path.push_back('/');
    
    const std::regex pattern("^meas-\\d.*\\.dat$");
    std::set<std::string> files;
    if(!get_file_names(path,files,pattern)){
        std::cout << "unable to open directory\n";
        return -1;
    }

    const auto first_file=*(files.begin());
    const auto second_file=*(files.erase(files.begin()));
    files.erase(files.begin());

    //read data from file1 and file2
    Vector3fVector features_1; //vector where each element is point_id-col-row where [col,row] is where it is observed in the image
    Vector10fVector appearances_1;
    Vector3fVector features_2;
    Vector10fVector appearances_2;
    if(!get_meas_content(path+first_file,appearances_1,features_1)){
        std::cout << "Unable to open file 1\n";
        return -1;
    }
    if(!get_meas_content(path+second_file,appearances_2,features_2)){
        std::cout << "Unable to open file 2\n";
        return -1;
    }
    
    // initialize a camera object
    std::vector<int> int_params; int_params.reserve(4); //z_near,z_far,cols,rows
    Eigen::Matrix3f k;
    Eigen::Isometry3f H;
    if(!get_camera_params(path+"camera.dat",int_params,k,H)){
        std::cout << "Unable to get camera parameters\n";
        return -1; 
    }
    Camera cam(int_params[3],int_params[2],int_params[0],int_params[1],k);
    std::cout << k << std::endl;

    return 0;
}