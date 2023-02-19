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

int main(int argc, char* argv[]) {
    // Data gathering --------
    if(argc < 2){
        std::cout << "Error: need path parameter to read data" << std::endl;
        return -1;
    }

    std::string path(argv[1]);
    if(path.back() != '/')
        path.push_back('/');
    
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
    std::vector<int> int_params; //z_near,z_far,cols,rows
    Eigen::Matrix3f k;
    Eigen::Isometry3f H;
    if(!get_camera_params(path+"camera.dat",int_params,k,H)){
        std::cout << "Unable to get camera parameters\n";
        return -1; 
    }
    // --------

    Camera cam(int_params[3],int_params[2],int_params[0],int_params[1],k);

    IsometryVector trajectory; trajectory.reserve(files.size());
    PICPSolver solver;
    solver.setKernelThreshold(10000);
    Eigen::Isometry3f X_curr=H.inverse();
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
    save_trajectory("trajectory_est.txt",trajectory,H);
    return 0;
 }