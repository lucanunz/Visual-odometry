//This file works on real data: it estimates the relative position between the first 2 set of measurements using epipolar geometry
#include <iostream>
#include "utils.h"
#include "files_utils.h"
#include "camera.h"

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

int main() {
    // Using real data 
    
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
    Vector3fVector world_points;
    Vector10fVector world_points_appearances;
    if(!get_meas_content(path+"world.dat",world_points_appearances,world_points,true)){
        std::cout << "Unable to open world file\n";
        return -1;     
    }

    //the pair is (ref_idx,curr_idx)
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
    std::cout << "R estimated:\n" << X.linear() << std::endl;
    std::cout << "t estimated: " << X.translation().transpose() << std::endl;
   
    return 0;
}