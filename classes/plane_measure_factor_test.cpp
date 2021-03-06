#include "plane_measure_factor_test.h"

PlaneMeasureFactorTest::PlaneMeasureFactorTest(Key key1, Key key2, 
                                       const Vector6& measured, const SharedNoiseModel& model):
                                       NoiseModelFactor2(model, key1, key2),
                                       measured_(measured){}

Vector PlaneMeasureFactorTest::evaluateError(const Pose3& pose, const Surfel & surfel,
                                         boost::optional<Matrix&> H1,
                                         boost::optional<Matrix&> H2) const{

    //With surfel expressed in global coordinate system and pose3,
    //We want to express the expected measurement with these inputs
    //h(pose, surfel)

    //With pose : R and t
    //With surfel : Snx, Sny, Snz, Sx, Sy, Sz
    Vector3 g_sn(surfel.nx, surfel.ny, surfel.nz);
    std::cout<<"g_sn"<<std::endl;
    std::cout<<"g_sn magnitude: "<<g_sn.transpose()*g_sn<<std::endl;
    Vector3 g_st(surfel.x, surfel.y, surfel.z);

    Matrix3 pose_R = pose.rotation().matrix();
    Vector3 pose_t = pose.translation().vector();
    
    Vector3 cur_sn = pose_R.transpose()*g_sn;
    Matrix3 cur_sn_skew = skewsym_matrix(cur_sn);

    Vector3 cur_st = pose_R.transpose()*(g_st - pose_t);
    Matrix3 cur_st_skew = skewsym_matrix(cur_st);

    Vector3 measure_n = measured_.segment(0,3);
    Vector3 measure_t = measured_.segment(3,3);


    Vector6 expected;
    expected.segment(0,3) = cur_sn;
    expected.segment(3,3) = cur_st;


    if(H1){
        Matrix H1_ = Matrix::Zero(6,6);
        H1_.block(0,0,3,3) = cur_sn_skew;
        H1_.block(3,0,3,3) = cur_st_skew;
        H1_.block(3,3,3,3) = -pose_R.transpose();

        *H1 = H1_;
    }
    if(H2){
        Matrix H2_ = Matrix::Zero(6,6);
        H2_.block(0,0,3,3) = pose_R.transpose();
        H2_.block(3,3,3,3) = pose_R.transpose();

        *H2 = H2_;
    }


    Vector6 result;
    result = expected - measured_;
    return result;
    
}

gtsam::Matrix3 PlaneMeasureFactorTest::skewsym_matrix(gtsam::Vector3 & vec) const{

    gtsam::Matrix3 result = gtsam::Matrix3::Zero(3,3);
    result(0,1) = -vec(2);
    result(0,2) = vec(1);
    result(1,0) = vec(2);
    result(1,2) = -vec(0);
    result(2,0) = -vec(1);
    result(2,1) = vec(0);
    return result;
}