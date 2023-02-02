// In this file picp is performed on the given data, assuming that the position of the
// points in the world is known and also data association in known
#include <iostream>
#include "utils.h"
#include "files_utils.h"
#include "camera.h"
#include "picp_solver.h"

Vector2fVector strip_id(const Vector3fVector& p_withid){
    Vector2fVector ret; ret.reserve(p_withid.size());

    for(const auto& el : p_withid)
        ret.push_back(el.tail<2>());
    
    return ret;
}

IntPairVector computeFakeCorrespondencesWorld(const Vector3fVector& image_points_withid){
    IntPairVector correspondences(image_points_withid.size());
    for(size_t i=0;i<image_points_withid.size();i++){
        correspondences[i]=IntPair(i,image_points_withid[i].x());
    }
    return correspondences;
}

 void save_gt_trajectory(const std::string& file_path){
    std::ifstream input_stream(file_path);
    std::string word;
    std::string line;
    Vector3fVector points;
    float n=0.f;
    if(!input_stream.is_open()){
        std::cout << "Unable to open " << file_path << std::endl;
        return;
    }
    while (std::getline(input_stream, line)) {
        std::stringstream ss(line);
        Eigen::Vector3f point;
        if(line.empty())
            continue;
        for (int i=0;i<4;i++)
            ss >> word;
        for (int i = 0; i < 2; i++) {
            ss >> n;
            point(i)=n;
        }
        point.z()=0.f;
        points.push_back(point);
    }
    input_stream.close();
    write_eigen_vectors_to_file("trajectory_gt.txt",points);
}

 int main(){
    // -------- data gathering
    const std::string path="/home/luca/vo_data/data/";
    save_gt_trajectory(path+"trajectory.dat");

    const std::regex pattern("^meas-\\d.*\\.dat$");
    std::set<std::string> files;
    if(!get_file_names(path,files,pattern)){
        std::cout << "unable to open directory\n";
        return -1;
    }
    Vector3fVector world_points;
    Vector10fVector world_points_appearances;
    if(!get_meas_content(path+"world.dat",world_points_appearances,world_points,true)){
        std::cout << "Unable to open world file\n";
        return -1;     
    }
    // initialize a camera object
    std::vector<int> int_params; //z_near,z_far,rows,cols
    Eigen::Matrix3f k;

    if(!get_camera_params(path+"camera.dat",int_params,k)){
        std::cout << "Unable to get camera parameters\n";
        return -1; 
    }
    // --------

    Camera cam(int_params[2],int_params[3],int_params[0],int_params[1],k);

    Eigen::Isometry3f X0; //relative pose of the 0 position of the camera in the world frame. Used to compare with the gt trajectory
    X0.linear() << 0.f, 0.f, 1.f,
            -1.f,0.f,0.f,
            0.f,-1.f,0.f;
    X0.translation() << 0.2f,0.f,0.f;

    VectorIsometry trajectory; trajectory.reserve(files.size());
    PICPSolver solver;
    solver.setKernelThreshold(10000);
    Eigen::Isometry3f X_curr=X0.inverse();
    for(const auto& file : files){
        Vector3fVector current_image_points_withid;
        Vector2fVector current_image_points;
        Vector10fVector current_appearances;
        if(!get_meas_content(path+file,current_appearances,current_image_points_withid)){
            std::cout << "Unable to open file " << path+file << std::endl;
            return -1;
        }
        
        for(auto& p : world_points)
            p=X_curr*p;

        current_image_points=strip_id(current_image_points_withid);
        IntPairVector correspondences_world=computeFakeCorrespondencesWorld(current_image_points_withid);

        cam.setWorldInCameraPose(Eigen::Isometry3f::Identity());
        solver.init(cam,world_points,current_image_points); //should find the current pose in the frame of the previous
        for(int i=0;i<1000;i++)
            solver.oneRound(correspondences_world,false);
        cam=solver.camera();
        trajectory.push_back(cam.worldInCameraPose());
        X_curr=cam.worldInCameraPose();
    }
    save_trajectory("trajectory_est.txt",trajectory);
    return 0;
 }