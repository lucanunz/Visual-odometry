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

bool get_camera_params(const std::string& file_path, std::vector<int>& int_params, Eigen::Matrix3f& k){
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
        else if(keyword=="z_near:" || keyword=="z_far:"|| keyword=="width:"||keyword=="height:"){
            ss >> n;
            int_params.push_back(n);
        }
    }
    input_stream.close();
    return true;
}
void save_trajectory(const std::string& file_path, const VectorIsometry& vector){
    std::ofstream output_file(file_path);
    if(!output_file.is_open()) {
        std::cout << "Error opening file" << std::endl;
        return;
    }
    Eigen::Isometry3f H;
    H.linear() << 0.f, 0.f, 1.f,
            -1.f,0.f,0.f,
            0.f,-1.f,0.f;
    H.translation() << 0.2f,0.f,0.f;
    for(size_t i=0;i<vector.size();i++){
        H=H*vector[i].inverse();
        output_file << H.translation().transpose() << std::endl;
    }
    
    output_file.close();
}