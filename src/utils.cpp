#include "utils.h"

bool get_file_names(const std::string& path, std::set<std::string>& files,const std::regex& pattern){
    DIR *dir;
    struct dirent *ent;
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
bool get_meas_content(const std::string& path_to_file, Vector10fVector& appearances, Vector3fVector& features){
    std::ifstream input_stream(path_to_file);
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
        Eigen::Vector3f feature;
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

bool get_camera_params(const std::string& path_to_file, std::vector<int>& int_params, Eigen::Matrix3f& k){
    std::ifstream input_stream(path_to_file);
    std::string keyword;
    std::string line;
    int n=0;
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

std::unordered_set<int> get_valid_ids(const Vector3fVector& p1, const Vector3fVector& p2){
    std::unordered_set<int> ids;
    for(const auto& el1 : p1){
        for(const auto & el2: p2){
            if(el2(0)>el1(0)) //measurements are ordered by the point id
                break;
            if(el2(0)==el1(0)){
                ids.insert(el1(0));
                break;
            }
        }
    }
    return ids;
}
void prune_projections(Vector3fVector& p1, Vector3fVector& p2,const std::unordered_set<int>& ids){
    p1.erase(std::remove_if(p1.begin(),
                                    p1.end(),
                                    [&](const Eigen::Vector3f& v)-> bool 
                                        { return (ids.find(v(0))==ids.end()); }), 
            p1.end());
    p2.erase(std::remove_if(p2.begin(),
                                p2.end(),
                                [&](const Eigen::Vector3f& v) -> bool 
                                    { return (ids.find(v(0))==ids.end()); }),
            p2.end());
}

template <typename SquareMatrixType_>
Eigen::Matrix<typename SquareMatrixType_::Scalar,
              SquareMatrixType_::RowsAtCompileTime, 1>

smallestEigenVector(const SquareMatrixType_& m) {
  Eigen::SelfAdjointEigenSolver<SquareMatrixType_> es;
  es.compute(m);
  return es.eigenvectors().col(0);
}

Eigen::Matrix3f estimate_essential(const Vector3fVector& p1_img, const Vector3fVector& p2_img){
    Matrix9f H=Matrix9f::Zero();
    const int n=p1_img.size();
    for(int i=0;i<n;i++){
        Eigen::Vector3f d1;
        d1 << p1_img[i].tail<2>(),1;
        Eigen::RowVector3f d2;
        d2 << p2_img[i].tail<2>().transpose(),1;
        Eigen::Matrix3f m=d1*d2;
        RowVector9f A;
        A << m(0,0),m(0,1),m(0,2),m(1,0),m(1,1),m(1,2),m(2,0),m(2,1),m(2,2);
        H.noalias()+=A.transpose()*A;
    }
    Vector9f e=smallestEigenVector(H);
    Eigen::Matrix3f E;
    E << e(0,0),e(0,1),e(0,2),
        e(1,0),e(1,1),e(1,2),
        e(2,0),e(2,1),e(2,2);
    return E;
}