#pragma once
#include <regex>
#include <dirent.h>
#include <set>

// writes in files the file names (in alphabetical order) found in path that match the regex
bool get_file_names(const char*& path, std::set<std::string>& files,const std::regex& pattern);
