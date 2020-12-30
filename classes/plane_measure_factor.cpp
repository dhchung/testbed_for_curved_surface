#include "plane_measure_factor.h"

PlaneMeasureFactor::PlaneMeasureFactor(Key key1, Key key2, 
                                       const Vector4& measured, const SharedNoiseModel& model):
                                       NoiseModelFactor2(model, key1, key2),
                                       measured_(measured){}

Vector PlaneMeasureFactor::evaluateError(const Pose3& pose, const Surfel & surfel,
                                         boost::optional<Matrix&> H1,
                                         boost::optional<Matrix&> H2) const{

    //With surfel expressed in global coordinate system and pose3,
    //We want to express the expected measurement with these inputs
    //h(pose, surfel)

    //With pose : R and t
    //With surfel : Snx, Sny, Snz, Sx, Sy, Sz
    Vector3 g_sn(surfel.nx, surfel.ny, surfel.nz);
    g_sn = g_sn/sqrt(g_sn.transpose()*g_sn);
    Vector3 g_st(surfel.x, surfel.y, surfel.z);

    Matrix3 pose_R = pose.rotation().matrix();
    Vector3 pose_t(pose.x(), pose.y(), pose.z());

    Vector3 cur_sn = pose_R.transpose()*g_sn;
    Matrix3 cur_sn_skew = skewsym_matrix(cur_sn);

    Vector3 measure_n = measured_.segment(0,3);
    double measure_d = measured_(3);


    Vector4 expected;
    expected.segment(0,3) = pose_R.transpose()* g_sn;
    expected(3) = g_sn.transpose()*(pose_t - g_st);


    if(H1){
        Matrix Dh1;

        Point3 normal_in(surfel.nx, surfel.ny, surfel.nz);
        Point3 normal_out = pose.rotation().unrotate(normal_in, Dh1);
        std::cout<<"DH1"<<std::endl;
        std::cout<<Dh1<<std::endl;

        Matrix H1_ = Matrix::Zero(4,6);
        H1_.block(0,0,3,3) = cur_sn_skew;
        // H1_(0,1) = -cur_sn(0);
        // H1_(0,2) = cur_sn(1);
        // H1_(1,0) = cur_sn(0);
        // H1_(1,2) = -cur_sn(2);
        // H1_(2,0) = -cur_sn(1);
        // H1_(2,1) = cur_sn(2);
        H1_.block(3,3,1,3) = g_sn.transpose();
        // H1_.block(3,3,1,3) = g_sn.transpose()*pose_R;

        std::cout<<"H1"<<std::endl;
        std::cout<<H1_<<std::endl;

        std::cout<<"g_sn"<<std::endl;
        std::cout<<g_sn.transpose()<<std::endl;
        std::cout<<"g_st"<<std::endl;
        std::cout<<g_st.transpose()<<std::endl;

        std::cout<<"pose_R"<<std::endl;
        std::cout<<pose_R<<std::endl;
        std::cout<<"pose_t"<<std::endl;
        std::cout<<pose_t.transpose()<<std::endl;

        std::cout<<"measure_n"<<std::endl;
        std::cout<<measure_n<<std::endl;
        
        std::cout<<"measure_d"<<std::endl;
        std::cout<<measure_d<<std::endl;
        *H1 = H1_;
    }
    if(H2){
        Matrix H2_ = Matrix::Zero(4,6);
        // H2_.block(0,0,3,3) = pose_R.transpose();
        H2_.block(0,0,3,3) = pose_R;
        H2_.block(3,0,1,3) = pose_t.transpose() - g_st.transpose();
        H2_.block(3,3,1,3) = -g_sn.transpose();
        // std::cout<<"H2"<<std::endl;
        // std::cout<<H2_<<std::endl;

        // std::cout<<"g_sn"<<std::endl;
        // std::cout<<g_sn.transpose()<<std::endl;
        // std::cout<<"g_st"<<std::endl;
        // std::cout<<g_st.transpose()<<std::endl;

        // std::cout<<"pose_R"<<std::endl;
        // std::cout<<pose_R<<std::endl;
        // std::cout<<"pose_t"<<std::endl;
        // std::cout<<pose_t.transpose()<<std::endl;

        // std::cout<<"measure_n"<<std::endl;
        // std::cout<<measure_n<<std::endl;
        
        // std::cout<<"measure_d"<<std::endl;
        // std::cout<<measure_d<<std::endl;



        *H2 = H2_;
    }


    // std::cout<<"___not in H"<<std::endl;

    // std::cout<<"g_sn"<<std::endl;
    // std::cout<<g_sn.transpose()<<std::endl;
    // std::cout<<"g_st"<<std::endl;
    // std::cout<<g_st.transpose()<<std::endl;

    // std::cout<<"pose_R"<<std::endl;
    // std::cout<<pose_R<<std::endl;
    // std::cout<<"pose_t"<<std::endl;
    // std::cout<<pose_t.transpose()<<std::endl;

    // std::cout<<"measure_n"<<std::endl;
    // std::cout<<measure_n<<std::endl;
    
    // std::cout<<"measure_d"<<std::endl;
    // std::cout<<measure_d<<std::endl;



    Vector4 result;
    result = expected - measured_;
    
    std::cout<<"Result"<<std::endl;
    std::cout<<result<<std::endl;

    std::cout<<"Expected"<<std::endl;
    std::cout<<expected<<std::endl;

    std::cout<<"measured"<<std::endl;
    std::cout<<measured_<<std::endl;

    std::cout<<"---------------------"<<std::endl;
    return result;




    //Measurement to global

    // Vector6 expected;
    // expected<<surfel.nx, surfel.ny, surfel.nz, surfel.x, surfel.y, surfel.z;

    // Vector3 local_coord(-measure_d/measure_n(0), 0.0, 0.0);


    // Vector6 measured;
    // measured.segment(0,3) = pose_R*measure_n;
    // measured.segment(3,3) = pose_R*local_coord + pose_t;


    // if(H1){
    //     Matrix H1_ = Matrix::Zero(6,6);
    //     // H1_.block(0,0,3,3) = pose_R*skewsym_matrix(measure_n);
    //     // H1_.block(0,3,3,3) = pose_R*skewsym_matrix(local_coord);
    //     H1_.block(0,0,3,3) = pose_R*skewsym_matrix(measure_n);
    //     H1_.block(0,3,3,3) = pose_R*skewsym_matrix(local_coord);
    //     H1_.block(3,3,3,3) = Matrix::Identity(3,3);
    //     *H1 = H1_;
    // }
    // if(H2){
    //     Matrix H2_ = Matrix::Zero(6,6);
    //     H2_ = Matrix::Identity(6,6);
    //     *H2 = H2_;
    // }

    // Vector6 result;
    // result = expected - measured;
    // std::cout<<"Result"<<std::endl;
    // std::cout<<result<<std::endl;

    // std::cout<<"Expected"<<std::endl;
    // std::cout<<expected<<std::endl;

    // std::cout<<"measured"<<std::endl;
    // std::cout<<measured<<std::endl;
    // return result;


}

gtsam::Matrix3 PlaneMeasureFactor::skewsym_matrix(gtsam::Vector3 & vec) const{

    gtsam::Matrix3 result = gtsam::Matrix3::Zero(3,3);
    result(0,1) = -vec(2);
    result(0,2) = vec(1);
    result(1,0) = vec(2);
    result(1,2) = -vec(0);
    result(2,0) = -vec(1);
    result(2,1) = vec(0);
    return result;
}