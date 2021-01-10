#include "surfel_measure_factor.h"

SurfelMeasureFactor::SurfelMeasureFactor(Key key1, Key key2, 
                                       const Vector6& measured, const SharedNoiseModel& model):
                                       NoiseModelFactor2(model, key1, key2),
                                       measured_(measured){}

Vector SurfelMeasureFactor::evaluateError(const Pose3& pose, const Surfel & surfel,
                                         boost::optional<Matrix&> H1,
                                         boost::optional<Matrix&> H2) const{

    //With surfel expressed in global coordinate system and pose3,
    //We want to express the expected measurement with these inputs
    //h(pose, surfel)

    //With pose : R and t
    //With surfel : Snx, Sny, Snz, Sx, Sy, Sz

    Unit3 g_sn(surfel.normal_dir);
    Point3 g_st(surfel.position);

    Unit3 m_n(measured_(0), measured_(1), measured_(2));
    Point3 m_p(measured_(3), measured_(4), measured_(5));


    Matrix HR;  //2*3
    Matrix Hp;  //2*2
    Matrix Dpose; //3*6
    Matrix Dpoint; //3*3

    Unit3 cur_sn = pose.rotation().unrotate(g_sn, HR, Hp);
    Point3 cur_st = pose.transformTo(g_st, Dpose, Dpoint);

    // std::cout<<"HR"<<std::endl;
    // std::cout<<HR<<std::endl;


    Matrix Hep;
    Matrix Heq;

    Vector2 error_normal = m_n.errorVector(cur_sn, Heq, Hep);
    Vector3 error_position = m_p.localCoordinates(cur_st);

    if(H1) {
        Matrix H1_ = Matrix::Zero(5,6);
        H1_.block(0,0,2,3) = HR;
        H1_.block(2,0,3,6) = Dpose;
        *H1 = H1_;
    }
    if(H2) {
        Matrix H2_ = Matrix::Zero(5,5);
        H2_.block(0,0,2,2) = Hep*Hp;
        H2_.block(2,2,3,3) = Dpoint;
        *H2 = H2_;
    }



    Vector5 result;

    result << error_normal, error_position;
    return result;


    // Vector3 g_sn(surfel.nx, surfel.ny, surfel.nz);
    // g_sn = g_sn/sqrt(g_sn.transpose()*g_sn);
    // Vector3 g_st(surfel.x, surfel.y, surfel.z);

    // Matrix3 pose_R = pose.rotation().matrix();
    // Vector3 pose_t(pose.x(), pose.y(), pose.z());

    // Vector3 cur_sn = pose_R.transpose()*g_sn;
    // Matrix3 cur_sn_skew = skewsym_matrix(cur_sn);

    // Vector3 measure_n = measured_.segment(0,3);
    // double measure_d = measured_(3);



    // Vector4 expected;
    // expected.segment(0,3) = pose_R.transpose()* g_sn;
    // expected(3) = g_sn.transpose()*(pose_t - g_st);


    // gtsam::Matrix H1_test;
    // gtsam::Matrix H2_test;

    // gtsam::Point3 normal_in(surfel.nx, surfel.ny, surfel.nz);
    // gtsam::Point3 normal_out = pose.rotation().unrotate(normal_in, H1_test, H2_test);


    // if(H1){
    //     Matrix H1_ = Matrix::Zero(4,6);
    //     // H1_.block(0,0,3,3) = cur_sn_skew;
    //     H1_.block(0,0,3,3) = H1_test;
    //     H1_.block(3,3,1,3) = g_sn.transpose();
    //     *H1 = H1_;
    // }
    // if(H2){
    //     Matrix H2_ = Matrix::Zero(4,6);
    //     H2_.block(0,0,3,3) = pose_R.transpose();
    //     H2_.block(3,0,1,3) = pose_t.transpose() - g_st.transpose();
    //     H2_.block(3,3,1,3) = -g_sn.transpose();
    //     *H2 = H2_;
    // }

    // Vector4 result;
    // result = expected - measured_;
    // return result;
}
