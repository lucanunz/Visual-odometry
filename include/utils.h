#pragma once
#include <regex>
#include <dirent.h>
#include <set>
#include <unordered_set>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <Eigen/Eigenvalues>
#include "defs.h"

// writes in files the file names (in alphabetical order) found in path that match the regex
bool get_file_names(const std::string& path, std::set<std::string>& files,const std::regex& pattern);

// writes in appearances and features the content of the file file_path
bool get_meas_content(const std::string& path_to_file, Vector10fVector& appearances, Vector3fVector& features);

// writes in int_params z_near,z_far,rows,cols and in k the camera matrix. These data are found in the file path_to_file
bool get_camera_params(const std::string& path_to_file, std::vector<int>& int_params, Eigen::Matrix3f& k);

// returns the ids only of the points that are present in both p1 and p2
std::unordered_set<int> get_valid_ids(const Vector3fVector& p1, const Vector3fVector& p2);

// given a set of valid ids, it removes from p1 and p2 the points whose id is not valid
void prune_projections(Vector3fVector& p1, Vector3fVector& p2,const std::unordered_set<int>& ids);

Eigen::Matrix3f estimate_essential(const Vector3fVector& p1_img, const Vector3fVector& p2_img);