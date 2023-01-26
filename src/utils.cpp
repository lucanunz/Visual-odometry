#include "utils.h"
bool get_file_names(const std::string& path, std::set<std::string>& files,const std::regex& pattern){
    DIR *dir;
    struct dirent *ent;
    std::string file_name;
    if ((dir = opendir(path.c_str())) != NULL) {
        while ((ent = readdir (dir)) != NULL) {
            file_name = ent->d_name;
            if (std::regex_search(file_name, pattern))
                files.insert(file_name);
        }
        closedir(dir);
        return true;
    } else 
        return false;
    
}
bool get_meas_content(const std::string& path_to_file, Vector10fVector& appearances, Vector3fVector& features){
    std::ifstream input_stream(path_to_file);
    std::string word;
    std::string line;
    float n=0.f;
    if(!input_stream.is_open())
        return false;

    Vector10f appearance;
    Eigen::Vector3f feature;
    // skip the first three lines
    for (int i = 0; i < 3; i++)
        std::getline(input_stream, line);

    while (std::getline(input_stream, line)) {
        std::stringstream ss(line);
        if(line.empty())
            continue;
        ss >> word; ss >> word;// skip the first 2 words
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
