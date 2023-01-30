#include <iostream>
#include "utils.h"
#include "files_utils.h"
#include "camera.h"

void computeFakeCorrespondences(IntPairVector& correspondences,
				const Vector2fVector reference_image_points,
				const Vector2fVector current_image_points){
    correspondences.resize(current_image_points.size());
    int num_correspondences=0;
    assert(reference_image_points.size()==current_image_points.size());
  
    for (size_t i=0; i<reference_image_points.size(); i++){
        const Eigen::Vector2f& reference_point=reference_image_points[i];
        const Eigen::Vector2f& current_point=current_image_points[i];
        IntPair& correspondence=correspondences[num_correspondences];
        if (reference_point.x()<0 || current_point.x()<0) //if one of them is the invalid point
            continue;
        correspondence.first=i;
        correspondence.second=i;
        num_correspondences++;
    }
    correspondences.resize(num_correspondences);
}

Vector2fVector strip_id(const Vector3fVector& p_withid){
    Vector2fVector ret; ret.reserve(p_withid.size());

    for(const auto& el : p_withid)
        ret.push_back(el.tail<2>());
    
    return ret;
}

int main() {
    // Real data ------------------------------------------------------------------------start
    
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
    Vector10fVector appearances_1;
    Vector3fVector current_image_points_withid;
    Vector10fVector appearances_2;
    if(!get_meas_content(path+first_file,appearances_1,reference_image_points_withid)){
        std::cout << "Unable to open file 1\n";
        return -1;
    }
    if(!get_meas_content(path+second_file,appearances_2,current_image_points_withid)){
        std::cout << "Unable to open file 2\n";
        return -1;
    }

    std::unordered_set<int> ids = get_valid_ids(reference_image_points_withid,current_image_points_withid);
    prune_projections(reference_image_points_withid,current_image_points_withid,ids);

    Vector2fVector reference_image_points=strip_id(reference_image_points_withid);
    Vector2fVector current_image_points=strip_id(current_image_points_withid);

    IntPairVector correspondences;
    computeFakeCorrespondences(correspondences, reference_image_points, current_image_points);
    // initialize a camera object
    std::vector<int> int_params; int_params.reserve(4); //z_near,z_far,rows,cols
    Eigen::Matrix3f k;

    if(!get_camera_params(path+"camera.dat",int_params,k)){
        std::cout << "Unable to get camera parameters\n";
        return -1; 
    }
    // Camera cam(int_params[2],int_params[3],int_params[0],int_params[1],k);
    // Camera cam2(int_params[2],int_params[3],int_params[0],int_params[1],k);

    //const Eigen::Matrix3f E=estimate_essential(k,correspondences,reference_image_points,current_image_points);
    const Eigen::Isometry3f X = estimate_transform(k, correspondences,reference_image_points,current_image_points);
    std::cout << "R estimated:\n" << X.linear() << std::endl;
    std::cout << "t estimated:\n" << X.translation().transpose() << std::endl;

    
    // Real data ----------------------------------------------------------------------------------------------end
}