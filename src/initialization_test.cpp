#include <iostream>
#include "utils.h"
#include "defs.h"
int main() {
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

    //std::cout << path+first_file << ", " << path+second_file << std::endl;

    Vector3fVector features_1; //vector where each element is point_id-col-row where [col,row] is where it is observed in the image
    Vector10fVector appearances_1;
    Vector3fVector features_2;
    Vector10fVector appearances_2;
    if(!get_meas_content(path+first_file,appearances_1,features_1)){
        std::cout << "Unable to open file 1\n";
        return -1;
    }
    if(!get_meas_content(path+second_file,appearances_2,features_2)){
        std::cout << "Unable to open file 2\n";
        return -1;
    }
    return 0;
}
