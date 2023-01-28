#include "utils.h"

Eigen::Matrix3f skew(const Eigen::Vector3f& v){
    Eigen::Matrix3f ret;
    ret << 0,-v(2),v(1),
        v(2),0,-v(0),
        -v(1),v(0),0;
    return ret;
}

void write_eigen_vectors_to_file(const std::string& file_path, const Vector3fVector& vectors) {
    std::ofstream output_file(file_path);
    if(!output_file.is_open()) {
        std::cout << "Error opening file" << std::endl;
        return;
    }
    for(const auto& vector : vectors)
        output_file << vector.transpose() << std::endl;
    
    output_file.close();
}
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

const Eigen::Matrix3f transform2essential(const Eigen::Isometry3f X){
    Eigen::Matrix3f E;
    E=X.linear().transpose()*skew(X.translation());
    return E;
}

const Eigen::Matrix3f estimate_essential(const Vector3fVector& p1_img, const Vector3fVector& p2_img, const Eigen::Matrix3f& k){
    if(p1_img.size()<8){
        std::cout << "Less than 8 points\n";
        return Eigen::Matrix3f::Zero();
    }
    if(p1_img.size() != p2_img.size()){
        std::cout << "Inconsistent points to estimate essential\n";
        return Eigen::Matrix3f::Zero();
    }
    Matrix9f H=Matrix9f::Zero();
    const auto iK=k.inverse();
    const int n=p1_img.size();
    for(int i=0;i<n;i++){
        Eigen::Vector3f d1;
        d1 << p1_img[i].tail<2>(),1;
        d1 = iK*d1;
        Eigen::Vector3f d2;
        d2 << p2_img[i].tail<2>(),1;
        d2 = iK*d2;
        const Eigen::Matrix3f m=d1*d2.transpose();
        RowVector9f A;
        A << m(0,0),m(0,1),m(0,2),m(1,0),m(1,1),m(1,2),m(2,0),m(2,1),m(2,2);
        H.noalias()+=A.transpose()*A;
    }
    Vector9f e=smallestEigenVector(H);
    Eigen::Matrix3f E;
    E << e(0),e(1),e(2),
        e(3),e(4),e(5),
        e(6),e(7),e(8);
    return E;
}

void generate_isometry3f(Eigen::Isometry3f& X){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-1.0f, 1.0f);

    Eigen::Vector3f a = Eigen::Vector3f::NullaryExpr(3,1,[&](){return dis(gen);});
    a.normalize();
    Eigen::AngleAxisf random_angle_axis(dis(gen),a);
    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f(random_angle_axis);

    X.linear()=rotation_matrix;
    X.translation()=Eigen::Vector3f::NullaryExpr(3,1,[&](){return dis(gen);});
}

void generate_points3d(const Eigen::Isometry3f& X,const int& num_points, Vector3fVector& p1, Vector3fVector& p2){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-5.0f, 5.0f);

    for(int i=0;i<num_points;i++){
        Eigen::Vector3f p = Eigen::Vector3f::NullaryExpr(3,1,[&](){return dis(gen);});
        p1.push_back(p);
        p2.push_back(X*p);
    }
}
