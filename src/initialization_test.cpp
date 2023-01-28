#include <iostream>
#include "utils.h"
#include "camera.h"
int main() {
    // Real data ------------------------------------------------------------------------start
    // const std::string path="/home/luca/vo_data/data/";
    // const std::regex pattern("^meas-\\d.*\\.dat$");
    // std::set<std::string> files;
    // if(!get_file_names(path,files,pattern)){
    //     std::cout << "unable to open directory\n";
    //     return -1;
    // }

    // const auto first_file=*(files.begin());
    // const auto second_file=*(files.erase(files.begin()));
    // files.erase(files.begin());

    // //read data from file1 and file2
    // Vector3fVector features_1; //vector where each element is point_id-col-row where [col,row] is where it is observed in the image
    // Vector10fVector appearances_1;
    // Vector3fVector features_2;
    // Vector10fVector appearances_2;
    // if(!get_meas_content(path+first_file,appearances_1,features_1)){
    //     std::cout << "Unable to open file 1\n";
    //     return -1;
    // }
    // if(!get_meas_content(path+second_file,appearances_2,features_2)){
    //     std::cout << "Unable to open file 2\n";
    //     return -1;
    // }

    // std::unordered_set<int> ids = get_valid_ids(features_1,features_2);
    // prune_projections(features_1,features_2,ids);
    // prune_projections(features_2,features_1,ids);

    // // initialize a camera object
    // std::vector<int> int_params; int_params.reserve(4); //z_near,z_far,rows,cols
    // Eigen::Matrix3f k;

    // if(!get_camera_params(path+"camera.dat",int_params,k)){
    //     std::cout << "Unable to get camera parameters\n";
    //     return -1; 
    // }
    // Camera cam(int_params[2],int_params[3],int_params[0],int_params[1],k);
    // Camera cam2(int_params[2],int_params[3],int_params[0],int_params[1],k);

    //Eigen::Matrix3f E=estimate_essential(features_1,features_2);
    //Eigen::JacobiSVD<Eigen::Matrix3f> svd(E);
    //std::cout << svd.singularValues() << std::endl;

    // Real data ----------------------------------------------------------------------------------------------end

    //Testing with synthetic data------------------------------------------------------------------------------start

    Eigen::Isometry3f X;
    generate_isometry3f(X);

    Vector3fVector p1;
    Vector3fVector p2;

    generate_points3d(X,90,p1,p2);
    write_eigen_vectors_to_file("p1.txt",p1);
    write_eigen_vectors_to_file("p2.txt",p2);

    Eigen::Matrix3f k;
    k << 100,0,320,
        0,100,240,
        0,0,1;
    Camera cam(480,640,0,10,k);
    Camera cam2(480,640,0,10,k);
    
    Vector3fVector p1_img;
    Vector3fVector p2_img;
    cam.projectPoints(p1_img,p1);
    cam2.projectPoints(p2_img,p2);

    auto valid_ids=get_valid_ids(p1_img,p2_img);
    // std::cout << "before:\n";
    // for(const auto& el : p1_img)
    //     std::cout << el.transpose() << std::endl;
    // std::cout << std::endl;
    // for(const auto& el : p2_img)
    //     std::cout << el.transpose() << std::endl;

    prune_projections(p1_img,p2_img,valid_ids);

    // std::cout << "after:\n";
    // for(const auto& el : p1_img)
    //     std::cout << el.transpose() << std::endl;
    // std::cout << std::endl;
    // for(const auto& el : p2_img)
    //     std::cout << el.transpose() << std::endl;
     const Eigen::Matrix3f E_gt = transform2essential(X);

     const Eigen::Matrix3f E_est=estimate_essential(p1_img,p2_img);
    
    //auto boh = k.transpose()*E_est*k;
    Eigen::JacobiSVD<Eigen::Matrix3f> svd_(E_est);
    std::cout << svd_.singularValues() << std::endl;
    std::cout << "E_gt:\n" << E_gt << std::endl << std::endl;
    std::cout << "E_est:\n" << E_est << std::endl << std::endl;
    for (int i=0;i<3;i++){
        for(int j=0;j<3;j++)
            std::cout << E_est(i,j)/E_gt(i,j) << " ";
        std::cout << std::endl;
    }
    //Testing with synthetic data------------------------------------------------------------------------------end
    return 0;
}

/* used for testing
    Vector3fVector a1={Eigen::Vector3f(2,12,23),Eigen::Vector3f(6,12,23),Eigen::Vector3f(7,12,23),Eigen::Vector3f(12,12,23),Eigen::Vector3f(22,12,23),Eigen::Vector3f(25,12,23)};
    Vector3fVector a2={Eigen::Vector3f(1,12,23),Eigen::Vector3f(2,12,25),Eigen::Vector3f(5,12,23),Eigen::Vector3f(11,12,23),Eigen::Vector3f(12,112,23),Eigen::Vector3f(25,12,23)};
    std::unordered_set<int> ids = get_valid_ids(a1,a2);
*/