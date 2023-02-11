#include "files_utils.h"

bool get_file_names(const std::string& path, std::set<std::string>& files,const std::regex& pattern){
    DIR *dir;
    struct dirent *ent;
    files.clear();
    if ((dir = opendir(path.c_str())) != NULL) {
        while ((ent = readdir (dir)) != NULL) {
            std::string file_name = ent->d_name;
            if (std::regex_search(file_name, pattern))
                files.insert(file_name);
        }
        closedir(dir);
        return true;
    } else 
        return false;
    
}
bool get_meas_content(const std::string& file_path, Vector10fVector& appearances, Vector3fVector& features,const bool& is_world){
    appearances.clear();
    features.clear();
    std::ifstream input_stream(file_path);
    std::string word;
    std::string line;
    float n=0.f;
    if(!input_stream.is_open())
        return false;

    // skip the first three lines
    if(!is_world)
        for (int i = 0; i < 3; i++)
            std::getline(input_stream, line);

    while (std::getline(input_stream, line)) {
        std::stringstream ss(line);
        Vector10f appearance;
        Eigen::Vector3f feature;
        if(line.empty())
            continue;
        ss >> word; //first word is always skipped. If we are reading world.dat we are skipping the id. Otherwise, we are skipping "point"
        if(!is_world){
            ss >> word;// skip the first 2 words
        }
        for (int i = 0; i < 13; i++) {
            ss >> n;
            if( i < 3)
                feature(i)=n;
            else{
                appearance(i-3)=n;
            }
        }
        features.push_back(feature);
        appearances.push_back(appearance);
    }
    input_stream.close();
    return true;
}
bool get_meas_content(const std::string& file_path, PointCloudVector<2>& points){
    points.clear();
    std::ifstream input_stream(file_path);
    std::string word;
    std::string line;
    float n=0.f;
    if(!input_stream.is_open())
        return false;

    // skip the first three lines
    for (int i = 0; i < 3; i++)
        std::getline(input_stream, line);

    while (std::getline(input_stream, line)) {
        std::stringstream ss(line);
        Vector10f appearance;
        Eigen::Vector2f feature;
        if(line.empty())
            continue;
        ss >> word;
        ss >> word; // skip the first 3 words
        ss >> word;
        
        for (int i = 0; i < 12; i++) {
            ss >> n;
            if( i < 2)
                feature(i)=n;
            else{
                appearance(i-2)=n;
            }
        }
        points.push_back(PointCloud<2>(feature,appearance));
    }
    input_stream.close();
    return true;
}
bool get_camera_params(const std::string& file_path, std::vector<int>& int_params, Eigen::Matrix3f& k, Eigen::Isometry3f& H){
    std::ifstream input_stream(file_path);
    std::string keyword;
    std::string line;
    int n=0;
    int_params.clear(); int_params.reserve(4);
    if(!input_stream.is_open())
        return false;

    while (std::getline(input_stream, line)) {
        std::stringstream ss(line);
        if(line.empty())
            continue;
        ss >> keyword;
        if (keyword=="camera"){
            for (int i=0;i<3;i++){
                std::getline(input_stream, line);
                std::stringstream ss_in(line);

                for(int j=0;j<3;j++)
                    ss_in >> k(i,j);
                
            }
        }
        else if (keyword=="cam_transform:"){
            for (int i=0;i<4;i++){
                std::getline(input_stream, line);
                std::stringstream ss_in(line);
                for(int j=0;j<4;j++)
                    ss_in >> H(i,j);
                
            }
        }
        else if(keyword=="z_near:" || keyword=="z_far:"|| keyword=="width:"||keyword=="height:"){
            ss >> n;
            int_params.push_back(n);
        }
    }
    input_stream.close();
    return true;
}

void save_trajectory(const std::string& file_path, const IsometryVector& vector, const Eigen::Isometry3f& cameraInRobot,const bool& save_rotation){
    std::ofstream output_file(file_path);
    if(!output_file.is_open()) {
        std::cout << "Unable to open " << file_path << " where to save the trajectory" << std::endl;
        return;
    }
    Eigen::Isometry3f H=Eigen::Isometry3f::Identity();

    for(size_t i=0;i<vector.size();i++){
        H=H*cameraInRobot*vector[i].inverse()*cameraInRobot.inverse(); //this gives the i-th pose of the robot in the world
        output_file << H.translation().transpose() << std::endl;
        if(save_rotation)
            output_file << H.linear() << std::endl;

    }
    
    output_file.close();
}

 bool save_gt_trajectory(const std::string& file_path){
    std::ifstream input_stream(file_path);
    std::string word;
    std::string line;
    Vector3fVector points;
    float n=0.f;
    if(!input_stream.is_open()){
        std::cout << "Unable to open " << file_path << std::endl;
        return false;
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
    return true;
}