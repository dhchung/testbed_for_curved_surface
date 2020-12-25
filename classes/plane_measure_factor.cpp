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
    Vector3 g_st(surfel.x, surfel.y, surfel.z);

    Matrix3 pose_R = pose.rotation().matrix();
    Vector3 pose_t(pose.x(), pose.y(), pose.z());

    Vector3 cur_sn = pose_R.transpose()*g_sn;
    Matrix3 cur_sn_skew = skewsym_matrix(cur_sn);

    Vector4 expected;
    expected.segment(0,3) = pose_R.transpose()* g_sn;
    expected(3) = g_sn.transpose()*(pose_t - g_st);

    if(H1){
        Matrix H1_ = Matrix::Zero(4,6);
        H1_.block(0,0,3,3) = cur_sn_skew;
        H1_.block(3,3,1,3) = g_sn.transpose();
        *H1 = H1_;
    }
    if(H2){
        Matrix H2_ = Matrix::Zero(4,6);
        H2_.block(0,0,3,3) = pose_R.transpose();
        H2_.block(3,0,1,3) = pose_t.transpose() - g_st.transpose();
        H2_.block(3,3,1,3) = -g_sn.transpose();
        *H2 = H2_;
    }

    Vector4 result;
    result = expected - measured_;
    return result;
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