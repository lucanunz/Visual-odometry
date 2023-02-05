#include <iostream>
#include <regex>
#include <dirent.h>
#include <set>
#include <unordered_set>
#include <fstream>
#include <string>
#include <sstream>
#include "defs.h"

//! writes Eigen vectors to a file, one for each row. They are to be used in gnuplot, e.g.
//! splot "p1.txt" u 1:2:3 w p pt 7,"p2.txt" u 1:2:3 w p pt 7 lt rgb "#FF0000"
//! @param file_path: path to the file where to write the vectors
//! @param vectors: vector of vectors of type Eigen to be writen in the file, one for each row
template <typename vec_type>
void write_eigen_vectors_to_file(const std::string& file_path, const std::vector<vec_type,Eigen::aligned_allocator<vec_type>>& vectors){
    std::ofstream output_file(file_path);
    if(!output_file.is_open()) {
        std::cout << "Error opening file" << std::endl;
        return;
    }
    for(const auto& vector : vectors)
        output_file << vector.transpose() << std::endl;
    
    output_file.close();
}

//! saves the trajectory stored in vector in the file_path specified
//! @param file_path: complete path where to save the file
//! @param vector: a vector of relative position vectors to save
void save_trajectory(const std::string& file_path, const VectorIsometry& vector);

//! writes in files the file names (in alphabetical order) found in path that match the regex
//! @param path: path where to look for the files
//! @param files: it will contain the file names that match the regex in alphabetical order
//! @param pattern: regular expression to be matched
bool get_file_names(const std::string& path, std::set<std::string>& files,const std::regex& pattern);

//! reads the file of measurements and writes the content in appearances and features
//! @param file_path: complete path of the file to read
//! @param appearances: will contain the appearance of each measurement found in the file
//! @param features: will contain id-col-row of each measurement in the file
//! @param is_world: if true, it means we are reading the file "world.dat". Otherwise, "meas-xxxxx.dat".
//! @returns false if it is not possible to open the specified file
bool get_meas_content(const std::string& file_path, Vector10fVector& appearances, Vector3fVector& features,const bool& is_world=false);

//! retrieves the camera parameters from the specified file
//! @param file_path: complete path of the file to read
//! @param int_params: will contain, in this order: z_near, z_far, width, height
//! @param k: will contain the 3x3 camera matrix
//! @returns false if it is not possible to open the specified file
bool get_camera_params(const std::string& file_path, std::vector<int>& int_params, Eigen::Matrix3f& k);