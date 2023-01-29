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

const IsometryPair essential2transformPair(const Eigen::Matrix3f& E){
    const Eigen::Matrix3f w((Eigen::Matrix3f() << 0.f, -1.f, 0.f,
                                                 1.f, 0.f, 0.f,
                                                 0.f, 0.f, 1.f).finished());
    //1st solution                                
    const Eigen::JacobiSVD<Eigen::Matrix3f> svd(E,Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f v=svd.matrixV();
    Eigen::Matrix3f u=svd.matrixU();
    Eigen::Matrix3f R1=v*w*u.transpose();
    if(R1.determinant()<0){
        const Eigen::JacobiSVD<Eigen::Matrix3f> svd(-E, Eigen::ComputeFullU | Eigen::ComputeFullV);
        v=svd.matrixV();
        u=svd.matrixU();
        R1=v*w*u.transpose();
    }
    Eigen::Isometry3f X1;
    X1.linear()=R1;
    Eigen::Matrix3f t_skew=R1*E;
    X1.translation()=Eigen::Vector3f(t_skew(2,1)-t_skew(1,2),t_skew(0,2)-t_skew(2,0),t_skew(1,0)-t_skew(0,1));

    //2nd solution
    Eigen::Matrix3f R2=v*w.transpose()*u.transpose();
    Eigen::Isometry3f X2;
    X2.linear()=R2;
    t_skew=R2*E;
    X2.translation()=Eigen::Vector3f(t_skew(2,1)-t_skew(1,2),t_skew(0,2)-t_skew(2,0),t_skew(1,0)-t_skew(0,1));
    IsometryPair X12(X1,X2);
    return X12;
}

bool triangulate_point(const Eigen::Vector3f& d1,const Eigen::Vector3f& d2,const Eigen::Vector3f& p2,Eigen::Vector3f& p){
    Eigen::Matrix<float,3,2> D;
    D.col(0)=-d1;
    D.col(1)=d2;
    const Eigen::Vector2f ss=-(D.transpose() * D).ldlt().solve(D.transpose() * p2);
    if(ss(0)<0 || ss(1)<0)
        return false;

    const Eigen::Vector3f p1_triangulated=ss(0)*d1;
    const Eigen::Vector3f p2_triangulated=p2+ss(1)*d2;
    //std::cout << "error: " << (p1_triangulated-p2_triangulated).norm() << std::endl;
    p=0.5f*(p1_triangulated+p2_triangulated);
    return true;
}

int triangulate_points(const Eigen::Matrix3f& k, const Eigen::Isometry3f& X, const Vector3fVector& p1_img, const Vector3fVector& p2_img, Vector4fVector& triangulated){
    const Eigen::Isometry3f iX = X.inverse();
    const Eigen::Matrix3f iK = k.inverse();
    const Eigen::Matrix3f iRiK = iX.linear()*iK;
    const Eigen::Vector3f t= iX.translation();
    const int num_points=p1_img.size();
    int n_success=0;
    for(int i=0;i<num_points;i++){
        Eigen::Vector3f d1;
        d1 << p1_img[i].tail<2>(),1;
        d1 = iK*d1;
        Eigen::Vector3f d2;
        d2 << p2_img[i].tail<2>(),1;
        d2 = iRiK*d2;
        Eigen::Vector3f p;
        if(triangulate_point(d1,d2,t,p)){
            n_success++;
            triangulated.push_back((Eigen::Vector4f() << p1_img[i].head<1>(),p).finished());
        }
    } 
    return n_success;
}
const Eigen::Isometry3f most_consistent_transform(const Eigen::Matrix3f k,const IsometryPair X12,const Vector3fVector& p1_img, const Vector3fVector& p2_img){
    int n_test=0,n_in_front=0;
    Eigen::Isometry3f X_best;
    Vector4fVector triang;
    Eigen::Isometry3f X_test=std::get<0>(X12);
    n_test=triangulate_points(k,X_test,p1_img,p2_img,triang);
    if (n_test > n_in_front){
        n_in_front=n_test;
        X_best=X_test;
    }
    X_test.translation()=-X_test.translation().eval();
    n_test=triangulate_points(k,X_test,p1_img,p2_img,triang);
    if (n_test > n_in_front){
        n_in_front=n_test;
        X_best=X_test;
    }

    X_test=std::get<1>(X12);
    n_test=triangulate_points(k,X_test,p1_img,p2_img,triang);
    if (n_test > n_in_front){
        n_in_front=n_test;
        X_best=X_test;
    }
    X_test.translation()=-X_test.translation().eval();
    n_test=triangulate_points(k,X_test,p1_img,p2_img,triang);
    if (n_test > n_in_front){
        n_in_front=n_test;
        X_best=X_test;
    }
    return X_best;
}