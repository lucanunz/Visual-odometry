#include "epipolar_utils.h"

const Eigen::Matrix3f transform2essential(const Eigen::Isometry3f X){
    Eigen::Matrix3f E;
    E=X.linear().transpose()*skew(X.translation());
    return E;
}

const Eigen::Matrix3f estimate_essential(const Eigen::Matrix3f& k, const IntPairVector& correspondences, const Vector2fVector& p1_img, const Vector2fVector& p2_img){
    if(correspondences.size()<8){
        std::cout << "Less than 8 points available to compute the essential matrix, aborting . . .\n";
        exit(-1);
    }
    //std::cout << correspondences.size() << std::endl << std::endl;
    Matrix9f H=Matrix9f::Zero();
    const auto iK=k.inverse();
    for (const IntPair& correspondence: correspondences){
        const int idx_first=correspondence.first;
        const int idx_second=correspondence.second;
        Eigen::Vector3f d1;
        d1 << p1_img[idx_first],1;
        d1 = iK*d1;

        Eigen::Vector3f d2;
        d2 << p2_img[idx_second],1;
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
    // const Eigen::JacobiSVD<Eigen::Matrix3f> svd(E,Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Eigen::Matrix3f E_constrained;
    // Eigen::DiagonalMatrix<float,3> Da(1.f,1.f,0.f);
    // E_constrained=svd.matrixU()*Da*svd.matrixV().transpose();
    // return E_constrained;
    return E;
}

Vector2fVector normalize(const Vector2fVector& p,Eigen::Matrix3f& T){
    Vector2fVector ret(p.size());
    float max_x=0.f,max_y=0.f;
    for(const auto& v : p){
        if(v.x()>max_x)
            max_x=v.x();
        if(v.y()>max_y)
            max_y=v.y();
    }

    for(size_t i=0;i<p.size();i++)
        ret[i] << p[i].x()/(max_x/2.f)-1.f,p[i].y()/(max_y/2.f)-1.f;

    T << 1.f/(max_x/2.f),0.f,-1.f,
        0.f,1.f/(max_y/2.f),-1.f,
        0.f,0.f,1.f;
    return ret;
}

Vector2fVector normalizeGauss(const Vector2fVector& p,Eigen::Matrix3f& T){
    const Eigen::Vector2f invalid_point(-1.f,-1.f);
    Vector2fVector p_valid; p_valid.reserve(p.size());
    std::unordered_set<int> invalid_ids;
    Eigen::Vector2f mu = Eigen::Vector2f::Zero();

    for(size_t i=0;i<p.size();i++){
        if(p[i] != invalid_point){
            p_valid.push_back(p[i]);
            mu+=p[i];
        }
        else
            invalid_ids.insert(i);
    }
    const size_t N=p_valid.size();
    mu = mu/N;
    Eigen::Matrix2f sigma=Eigen::Matrix2f::Zero();
    for(const auto& v : p_valid){
        Eigen::Vector2f c=v-mu;
        sigma+=c*c.transpose();
    }
    sigma=sigma*1/(N-1);
    Eigen::Matrix2f chol(sigma.llt().matrixL());

    T=Eigen::Matrix3f::Identity();
    T.block<2,2>(0,0)=chol.inverse();
    T.block<2,1>(0,2)=-T.block<2,2>(0,0)*mu;

    Vector2fVector ret=p;
    for(size_t i=0;i<p.size();i++)
        if(invalid_ids.find(i)==invalid_ids.end())
            ret[i]=T.block<2,2>(0,0)*p[i]+T.block<2,1>(0,2);     
    
    return ret;
}

const Eigen::Matrix3f estimate_fundamental(const IntPairVector& correspondences, 
                                            const Vector2fVector& p1_img, const Vector2fVector& p2_img){
    if(correspondences.size()<8){
        std::cout << "Less than 8 points available to compute the fundamental matrix, aborting . . .\n";
        exit(-1);
    }
    Eigen::Matrix3f T1,T2; //conditioning matrices
    //coordinate points normalization in [-1,1]
    Vector2fVector p1_img_norm=normalize(p1_img,T1);
    Vector2fVector p2_img_norm=normalize(p2_img,T2);

    const int N=correspondences.size();
    Eigen::MatrixXf A(N,9);

    for (int i=0;i<N;i++){
        const int idx_first=correspondences[i].first;
        const int idx_second=correspondences[i].second;
        Eigen::Vector3f d1;
        d1 << p1_img_norm[idx_first],1;
        Eigen::Vector3f d2;
        d2 << p2_img_norm[idx_second],1;
        const Eigen::Matrix3f m=d1*d2.transpose();
        A.row(i)<<m(0,0),m(0,1),m(0,2),m(1,0),m(1,1),m(1,2),m(2,0),m(2,1),m(2,2);
    }
    const Eigen::JacobiSVD<Eigen::MatrixXf> svd(A,Eigen::ComputeThinV);
    Vector9f v=svd.matrixV().col(8);
    Eigen::Matrix3f Fa;
    Fa << v(0),v(1),v(2),
            v(3),v(4),v(5),
            v(6),v(7),v(8);
    const Eigen::JacobiSVD<Eigen::Matrix3f> svd_Fa(Fa,Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::DiagonalMatrix<float,3> Da;
    Da.diagonal() << svd_Fa.singularValues().head<2>(),0.f;

    // impose fundamental to have rk 2
    Eigen::Matrix3f F;
    F=svd_Fa.matrixU()*Da*svd_Fa.matrixV().transpose();

    // undo the effect of coordinates normalization
    return T1.transpose()*F*T2;
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
    Eigen::Isometry3f X1=Eigen::Isometry3f::Identity();
    X1.linear()=R1;
    Eigen::Matrix3f t_skew=R1*E;
    X1.translation()=Eigen::Vector3f(t_skew(2,1),t_skew(0,2),t_skew(1,0));

    //2nd solution
    Eigen::Matrix3f R2=v*w.transpose()*u.transpose();
    Eigen::Isometry3f X2=Eigen::Isometry3f::Identity();
    X2.linear()=R2;
    t_skew=R2*E;
    X2.translation()=Eigen::Vector3f(t_skew(2,1),t_skew(0,2),t_skew(1,0));
    IsometryPair X12(X1,X2);
    return X12;
}

const Eigen::Isometry3f estimate_transform(const Eigen::Matrix3f k, const IntPairVector& correspondences, 
                                            const Vector2fVector& p1_img, const Vector2fVector& p2_img){

    Eigen::Matrix3f F=estimate_fundamental(correspondences,p1_img,p2_img);
    Eigen::Matrix3f E=k.transpose()*F*k;    
    const IsometryPair X12=essential2transformPair(E);

    int n_test=0,n_in_front=0;
    Eigen::Isometry3f X_best=Eigen::Isometry3f::Identity();
    Vector3fVector triang;
    
    Eigen::Isometry3f X_test=std::get<0>(X12);
    n_test=triangulate_points(k,X_test,correspondences,p1_img,p2_img,triang);
    if (n_test > n_in_front){
        n_in_front=n_test;
        X_best=X_test;
    }
    X_test.translation()=-X_test.translation().eval();
    n_test=triangulate_points(k,X_test,correspondences,p1_img,p2_img,triang);
    if (n_test > n_in_front){
        n_in_front=n_test;
        X_best=X_test;
    }

    X_test=std::get<1>(X12);
    n_test=triangulate_points(k,X_test,correspondences,p1_img,p2_img,triang);
    if (n_test > n_in_front){
        n_in_front=n_test;
        X_best=X_test;
    }
    X_test.translation()=-X_test.translation().eval();
    n_test=triangulate_points(k,X_test,correspondences,p1_img,p2_img,triang);
    if (n_test > n_in_front){
        n_in_front=n_test;
        X_best=X_test;
    }
    return X_best;
}