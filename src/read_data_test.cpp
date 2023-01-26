#include <iostream>
#include "utils.h"

int main() {
    const std::string path="/home/luca/vo_data/data/";
    const std::regex pattern("^meas-\\d.*\\.dat$");
    std::set<std::string> files;
    if(!get_file_names(path,files,pattern)){
        std::cout << "unable to open directory\n";
        return -1;
    }
    for (const auto& file : files)
        std::cout << file << std::endl;
    

    return 0;
}