#include "utils.h"
bool get_file_names(const char*& path, std::set<std::string>& files,const std::regex& pattern){
    DIR *dir;
    struct dirent *ent;

    if ((dir = opendir(path)) != NULL) {
        while ((ent = readdir (dir)) != NULL) {
            std::string file_name = ent->d_name;
            if (std::regex_search(file_name, pattern)) {
                files.insert(file_name);
            }
        }
        closedir (dir);
        return true;
    } else 
        return false;
    
}