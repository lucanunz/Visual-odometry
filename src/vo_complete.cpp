//This file works on real data: it estimates the relative position between the first 2 set of measurements using epipolar geometry
//triangulates to find a set of world points, and then iteratively performs picp between subsequent poses.
#include <iostream>
#include "utils.h"
#include "files_utils.h"
#include "camera.h"
#include "picp_solver.h"
#include "eigen_kdtree.h"
#include <unordered_map>
#include "epipolar_utils.h"

IntPairVector compute_correspondences_images(const Vector10fVector& appearances1, const Vector10fVector& appearances2){
    using ContainerType = Vector11fVector;
    using TreeNodeType = TreeNode_<ContainerType::iterator>;
    IntPair nN=std::minmax(appearances1.size(),appearances2.size());
    IntPairVector correspondences; correspondences.reserve(nN.first);
    ContainerType kd_points(nN.second);
    ContainerType query_points(nN.first);

    if(nN.second == (int) appearances1.size()){
        for (size_t i=0;i<kd_points.size();i++)
            kd_points[i] = (Vector11f() << float(i),appearances1[i]).finished();

        for (size_t i=0;i<query_points.size();i++)
            query_points[i] = (Vector11f() << float(i),appearances2[i]).finished();
    }
    else{
        for (size_t i=0;i<kd_points.size();i++)
            kd_points[i] = (Vector11f() << float(i),appearances2[i]).finished();
        
        for (size_t i=0;i<query_points.size();i++)
            query_points[i] = (Vector11f() << float(i),appearances1[i]).finished();
    }

    TreeNodeType  kd_tree(kd_points.begin(), kd_points.end(), 10);

    for(const auto& p : query_points){
        Vector11f* match_full=kd_tree.bestMatchFull(p, 0.1f);
        if(match_full){
            if(nN.second == (int) appearances1.size())
                correspondences.push_back(IntPair((*match_full)(0),p(0)));
            else
                correspondences.push_back(IntPair(p(0),(*match_full)(0)));
        }
        
    }

    return correspondences;
}
// Hash approach 
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
// IntPairVector compute_correspondences_map(const PointCloudVector<3>& map,const PointCloudVector<2>& measurements){
//     std::unordered_map<Vector10f, int, matrix_hash<Vector10f>> hash_map;
//     IntPairVector correspondences; correspondences.reserve(measurements.size());
//     for (size_t i=0;i<map.appearances().size();i++)
//         hash_map[map.appearances().at(i)] = i;
    
//     for (size_t i=0;i<measurements.size();i++) {
//         auto it=hash_map.find(measurements.appearances().at(i));

//         if (it!=hash_map.end())
//             correspondences.push_back(IntPair(i,it->second));
        
//     }
//     return correspondences;
// }

// kd-tree solution
IntPairVector compute_correspondences_map(const PointCloudVector<3>& map,const PointCloudVector<2>& measurements){
    using ContainerType = Vector11fVector;
    using TreeNodeType = TreeNode_<ContainerType::iterator>;

    IntPairVector correspondences; correspondences.reserve(measurements.size());
    ContainerType kd_points(map.size());
    ContainerType query_points(measurements.size());

    for (size_t i=0;i<kd_points.size();i++)
        kd_points[i] = (Vector11f() << float(i),map.appearances().at(i)).finished(); 
    for (size_t i=0;i<query_points.size();i++)
        query_points[i] = (Vector11f() << float(i),measurements.appearances().at(i)).finished();

    TreeNodeType  kd_tree(kd_points.begin(), kd_points.end(), 10);

    for(const auto& p : query_points){
        Vector11f* match_full=kd_tree.bestMatchFull(p, 0.1f);
        if(match_full){
            correspondences.push_back(IntPair(p(0),(*match_full)(0)));
        }

    }
    return correspondences;
}
// Exhaustive search
// IntPairVector compute_correspondences_map(const PointCloudVector<3>& map,const PointCloudVector<2>& measurements){

//     IntPairVector correspondences; correspondences.reserve(measurements.size());
    
//     for(size_t i=0;i<map.size();i++){
//         const Vector10f app=map.appearances().at(i);
//         for(size_t j=0;j<measurements.size();j++){
//             if(app==measurements.appearances().at(j)){
//                 correspondences.push_back(IntPair(j,i));
//                 break;
//             }
//         }
//     }


//     return correspondences;
// }
int main(int argc, char* argv[]) {
    // Using real data 
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

    const auto first_file=*(files.begin());
    const auto second_file=*(files.erase(files.begin()));
    files.erase(files.begin());

    //read data from file1 and file2

    PointCloudVector<2> reference_pc;
    PointCloudVector<2> current_pc;
    if(!get_meas_content(path+first_file,reference_pc)){
        std::cout << "Unable to open measurement file 0\n";
        return -1;
    }
    if(!get_meas_content(path+second_file,current_pc)){
        std::cout << "Unable to open file measurement file 1\n";
        return -1;
    }
    Vector3fVector world_points;
    Vector10fVector world_points_appearances;
    if(!get_meas_content(path+"world.dat",world_points_appearances,world_points,true)){
        std::cout << "Unable to open world file\n";
        return -1;     
    }
    write_eigen_vectors_to_file("world.txt",world_points);
    //the pairs are (ref_idx,curr_idx)
    IntPairVector correspondences_imgs = compute_correspondences_images(reference_pc.appearances(),current_pc.appearances());
    // std::cout << correspondences_imgs.size() << std::endl;
    // initialize a camera object
    std::vector<int> int_params; //z_near,z_far,cols,rows
    Eigen::Matrix3f k;
    Eigen::Isometry3f H;
    if(!get_camera_params(path+"camera.dat",int_params,k,H)){
        std::cout << "Unable to get camera parameters\n";
        return -1; 
    }
    Camera cam(int_params[3],int_params[2],int_params[0],int_params[1],k);

    const Eigen::Isometry3f X = estimate_transform(cam.cameraMatrix(), correspondences_imgs, reference_pc.points(), current_pc.points());

    PointCloudVector<3> triangulated_pc;
    IntPairVector correspondences_world;
    // Eigen::Isometry3f X_gt=Eigen::Isometry3f::Identity();
    // X_gt.translation() << 0.f,0.f,-0.200426f;
    triangulate_points(k,X,correspondences_imgs,reference_pc,
                        current_pc,triangulated_pc,correspondences_world); // At this stage correspondences_world contains the pairs (curr_idx,world_idx)

    // The estimated transform X is the pose 00000 in frame 00001. "triangulated" are points expressed in 00000.

    IsometryVector trajectory; trajectory.reserve(files.size()+2);
    trajectory.push_back(Eigen::Isometry3f::Identity());
    trajectory.push_back(X);
    PICPSolver solver;
    solver.setKernelThreshold(10000);

    reference_pc=current_pc;
    //given the above swaps, now correspondences_world actually contains (ref_idx,world_idx)
    double t_start=0;
    double t_end=0;
    std::ofstream time_file("time_kd.txt");
    PointCloudVector<3> triangulated_transformed;
    PointCloudVector<3> map;
    PointCloudVector<3> points_for_picp;
    map.update(triangulated_pc);
    Eigen::Isometry3f history=X.inverse();
    for(const auto& file : files){

        if(!get_meas_content(path+file,current_pc)){
            std::cout << "Unable to open file " << path+file << std::endl;
            return -1;
        }
        correspondences_imgs = compute_correspondences_images(reference_pc.appearances(),current_pc.appearances());
        t_start=getTime();
        IntPairVector correspondences_map=compute_correspondences_map(map,current_pc);
        t_end=getTime();

        points_for_picp=history.inverse()*map;

        cam.setWorldInCameraPose(Eigen::Isometry3f::Identity());
        solver.init(cam,points_for_picp.points(),current_pc.points()); //should find the current pose in the frame of the previous
        for(int i=0;i<1000;i++)
            solver.oneRound(correspondences_map,false);
        cam=solver.camera();

        trajectory.push_back(cam.worldInCameraPose());
        std::cout << "******************* File " << file << std::endl;
        std::cout << cam.worldInCameraPose().linear() << std::endl;
        std::cout << cam.worldInCameraPose().translation().transpose() << std::endl;
        triangulate_points(k,cam.worldInCameraPose(),correspondences_imgs,reference_pc,
                        current_pc,triangulated_pc,correspondences_world);

        map.update(history*triangulated_pc);
        history=history*cam.worldInCameraPose().inverse();
        reference_pc=current_pc;

        std::cout << "correspondences search took: " << (t_end-t_start) << " ms" << std::endl;
        time_file << t_end-t_start << std::endl;
    }
    time_file.close();

    map=H*map;
    write_eigen_vectors_to_file("map.txt",map.points());
    write_eigen_vectors_to_file("map_appearances.txt",map.appearances());
    save_trajectory("trajectory_est_complete.txt",trajectory,H);
    save_trajectory("trajectory_est_data.txt",trajectory,H,true);
    return 0;
}