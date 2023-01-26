#pragma once
#include <regex>
#include <dirent.h>
#include <set>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include "defs.h"
// writes in files the file names (in alphabetical order) found in path that match the regex
bool get_file_names(const std::string& path, std::set<std::string>& files,const std::regex& pattern);

// writes in appearances and features the content of the file file_path
bool get_meas_content(const std::string& path_to_file, Vector10fVector& appearances, Vector3fVector& features);