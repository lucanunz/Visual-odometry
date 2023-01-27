#include <iostream>
#include "utils.h"
#include "defs.h"
#include "camera.h"
int main() {
    const std::string path="/home/luca/vo_data/data/";
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

    std::unordered_set<int> ids = get_valid_ids(features_1,features_2);
    prune_projections(features_1,features_2,ids);
    prune_projections(features_2,features_1,ids);

    // initialize a camera object
    std::vector<int> int_params; int_params.reserve(4); //z_near,z_far,rows,cols
    Eigen::Matrix3f k;

    if(!get_camera_params(path+"camera.dat",int_params,k)){
        std::cout << "Unable to get camera parameters\n";
        return -1; 
    }
    Camera cam(int_params[2],int_params[3],int_params[0],int_params[1],k);
    Eigen::Matrix3f E=estimate_essential(features_1,features_2);

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(E);
    std::cout << svd.singularValues() << std::endl;

    return 0;
}

/* used for testing
    Vector3fVector a1={Eigen::Vector3f(2,12,23),Eigen::Vector3f(6,12,23),Eigen::Vector3f(7,12,23),Eigen::Vector3f(12,12,23),Eigen::Vector3f(22,12,23),Eigen::Vector3f(25,12,23)};
    Vector3fVector a2={Eigen::Vector3f(1,12,23),Eigen::Vector3f(2,12,25),Eigen::Vector3f(5,12,23),Eigen::Vector3f(11,12,23),Eigen::Vector3f(12,112,23),Eigen::Vector3f(25,12,23)};
    std::unordered_set<int> ids = get_valid_ids(a1,a2);
*/