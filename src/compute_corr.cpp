//This file works on real data: it estimates the relative position between the first 2 set of measurements using epipolar geometry
#include <iostream>
#include "utils.h"
#include "files_utils.h"
#include "camera.h"
#include "eigen_kdtree.h"
#include <unordered_map>
Vector2fVector strip_id(const Vector3fVector& p_withid){
    Vector2fVector ret; ret.reserve(p_withid.size());

    for(const auto& el : p_withid)
        ret.push_back(el.tail<2>());
    
    return ret;
}
// Solution using hash. Unused, fragile wrt noise since we are hashing floats
// template<typename T>
// struct matrix_hash : std::unary_function<T, size_t> {
//   std::size_t operator()(T const& matrix) const {
//     size_t seed = 0;
//     for (size_t i = 0; i < matrix.size(); ++i) {
//       auto elem = *(matrix.data() + i);
//       seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
//     }
//     return seed;
//   }
// };
// IntPairVector compute_correspondences_imgs(const Vector10fVector& appearances1, const Vector10fVector& appearances2){
//     std::unordered_map<Vector10f, int, matrix_hash<Vector10f>> map;
//     IntPairVector correspondences; correspondences.reserve(std::min(appearances1.size(),appearances2.size()));
//     for (size_t i=0;i<appearances1.size();i++)
//         map[appearances1[i]] = i;
    
//     for (size_t i=0;i<appearances2.size();i++) {
//         auto it=map.find(appearances2[i]);

//         if (it!=map.end())
//             correspondences.push_back(IntPair(it->second, i));
        
//     }
//     return correspondences;
// }
IntPairVector compute_correspondences_imgs(const Vector10fVector& appearances1, const Vector10fVector& appearances2){
    using ContainerType = Vector11fVector;
    using TreeNodeType = TreeNode_<ContainerType::iterator>;
    IntPairVector correspondences; correspondences.reserve(std::min(appearances1.size(),appearances2.size()));
    
    ContainerType kd_points(appearances1.size());
    for (size_t i=0;i<kd_points.size();i++)
        kd_points[i] = (Vector11f() << float(i),appearances1[i]).finished();

    TreeNodeType  kd_tree(kd_points.begin(), kd_points.end(), 10);

    ContainerType query_points(appearances2.size());
    for (size_t i=0;i<query_points.size();i++)
        query_points[i] = (Vector11f() << float(i),appearances2[i]).finished();
    
    for(const auto& p : query_points){
        //std::cout << p.transpose() << std::endl;
        TreeNodeType::AnswerType neighbors;
        Vector11f* match_full=kd_tree.bestMatchFull(p, 0.1f);
        if(match_full)
            correspondences.push_back(IntPair((*match_full)(0),p(0)));
        
    }
    return correspondences;
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

    //the pair is (ref_idx,curr_idx)
    IntPairVector correspondences_imgs = compute_correspondences_imgs(reference_appearances,current_appearances);
    IntPairVector correspondences_imgs_gt = extract_correspondences_images(reference_image_points_withid,current_image_points_withid);
    std::cout << "sizes: " << correspondences_imgs_gt.size() << ", " << correspondences_imgs.size() << "\n";
    for(size_t i=0;i<correspondences_imgs_gt.size();i++)
        std::cout << "gt: " << correspondences_imgs_gt[i].first << ", " << correspondences_imgs_gt[i].second << "\nest: " << correspondences_imgs[i].first << ", " << correspondences_imgs[i].second << "\n**********\n";
    return 0;
}