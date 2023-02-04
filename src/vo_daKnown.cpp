//This file works on real data: it estimates the relative position between the first 2 set of measurements using epipolar geometry
//triangulates to find a set of world points, and then iteratively performs picp between subsequent poses. Data association is given
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

IntPairVector extract_correspondences_images(const Vector3fVector& reference_image_points_withid,const Vector3fVector& current_image_points_withid){
    IntPairVector correspondences; correspondences.reserve(current_image_points_withid.size());

    for(size_t i=0;i<reference_image_points_withid.size();i++){
        for(size_t j=0;j<current_image_points_withid.size();j++){
            if(current_image_points_withid[j].x()>reference_image_points_withid[i].x()) //measurements are ordered by the point id
                break;
            if(current_image_points_withid[j].x()==reference_image_points_withid[i].x()){
                correspondences.push_back(IntPair(i,j));
                break;
            }
        }
    }
    return correspondences;
}

// correspondences_imgs are (ref_idx,curr_idx), correspondences_world are (ref_idx,world_idx). We extract the pairs (curr_idx,world_idx)
IntPairVector extract_correspondences_world(const IntPairVector& correspondences_imgs,const IntPairVector& correspondences_world){
    IntPairVector correspondences; correspondences.reserve(correspondences_imgs.size());

    for(size_t i=0;i<correspondences_imgs.size();i++){
        const int idx_ref=correspondences_imgs[i].first;
        for(size_t j=0;j<correspondences_world.size();j++){
            if(correspondences_world[j].first==idx_ref){
                correspondences.push_back(IntPair(correspondences_imgs[i].second,correspondences_world[j].second));
                break;
            }
        }
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
int main() {
    // Using real data 

    const std::string path="/home/luca/vo_data/data/";
    save_gt_trajectory(path+"trajectory.dat");
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
    Vector3fVector reference_image_points_withid; //vector where each element is point_id-col-row where [col,row] is where it is observed in the image
    Vector10fVector reference_appearances;
    Vector3fVector current_image_points_withid;
    Vector10fVector current_appearances;

    if(!get_meas_content(path+first_file,reference_appearances,reference_image_points_withid)){
        std::cout << "Unable to open file 1\n";
        return -1;
    }
    if(!get_meas_content(path+second_file,current_appearances,current_image_points_withid)){
        std::cout << "Unable to open file 2\n";
        return -1;
    }

    //the pairs are (ref_idx,curr_idx)
    IntPairVector correspondences_imgs = extract_correspondences_images(reference_image_points_withid,current_image_points_withid);

    Vector2fVector reference_image_points=strip_id(reference_image_points_withid);
    Vector2fVector current_image_points=strip_id(current_image_points_withid);

    // initialize a camera object
    std::vector<int> int_params; //z_near,z_far,rows,cols
    Eigen::Matrix3f k;

    if(!get_camera_params(path+"camera.dat",int_params,k)){
        std::cout << "Unable to get camera parameters\n";
        return -1; 
    }
    Camera cam(int_params[2],int_params[3],int_params[0],int_params[1],k);

    const Eigen::Isometry3f X = estimate_transform(cam.cameraMatrix(), correspondences_imgs, reference_image_points, current_image_points);

    Vector3fVector triangulated;
    IntPairVector correspondences_world;
    // Eigen::Isometry3f X_gt=Eigen::Isometry3f::Identity();
    // X_gt.translation() << 0.f,0.f,-0.200426f;
    triangulate_points(k,X,correspondences_imgs,reference_image_points,
                        current_image_points,triangulated,correspondences_world); // At this stage correspondences_world contains the pairs (curr_idx,world_idx)

    // The estimated transform X is the pose 00000 in frame 00001. "triangulated" are points expressed in 00000.

    VectorIsometry trajectory; trajectory.reserve(files.size()+2);
    trajectory.push_back(Eigen::Isometry3f::Identity());
    trajectory.push_back(X);
    PICPSolver solver;
    solver.setKernelThreshold(10000);
    Eigen::Isometry3f X_curr=X;

    reference_image_points=current_image_points;
    reference_image_points_withid=current_image_points_withid;
    reference_appearances=current_appearances;
    //given the above swaps, now correspondences_world actually contains (ref_idx,world_idx)

    for(const auto& file : files){

        if(!get_meas_content(path+file,current_appearances,current_image_points_withid)){
            std::cout << "Unable to open file " << path+file << std::endl;
            return -1;
        }

        correspondences_imgs = extract_correspondences_images(reference_image_points_withid,current_image_points_withid);
        current_image_points=strip_id(current_image_points_withid);
        Vector2fVector reference_image_points=strip_id(reference_image_points_withid);
        correspondences_world=extract_correspondences_world(correspondences_imgs,correspondences_world);

        for(auto& p : triangulated)
            p=X_curr*p;

        cam.setWorldInCameraPose(Eigen::Isometry3f::Identity());
        solver.init(cam,triangulated,current_image_points); //should find the current pose in the frame of the previous
        for(int i=0;i<1000;i++)
            solver.oneRound(correspondences_world,false);
        cam=solver.camera();

        trajectory.push_back(cam.worldInCameraPose());
        X_curr=cam.worldInCameraPose();
        std::cout << "******************* File " << file << std::endl;
        std::cout << cam.worldInCameraPose().linear() << std::endl;
        std::cout << cam.worldInCameraPose().translation().transpose() << std::endl;
        triangulate_points(k,cam.worldInCameraPose(),correspondences_imgs,reference_image_points,
                        current_image_points,triangulated,correspondences_world);
        reference_image_points=current_image_points;
        reference_image_points_withid=current_image_points_withid;
        reference_appearances=current_appearances;
    }
    save_trajectory("trajectory_est_noWorld.txt",trajectory);
    return 0;
}