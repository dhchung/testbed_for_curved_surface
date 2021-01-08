#include "plane_measure_factor_new.h"

PlaneMeasureFactorNew::PlaneMeasureFactorNew(Key key1, Key key2, 
                                             const Vector6& measured, const SharedNoiseModel& model):
                                             NoiseModelFactor2(model, key1, key2),
                                             measured_(measured){}

Vector PlaneMeasureFactorNew::evaluateError(const Pose3& pose, const Surfel & surfel,
                                         boost::optional<Matrix&> H1,
                                         boost::optional<Matrix&> H2) const{

    //With surfel expressed in global coordinate system and pose3,
    //We want to express the expected measurement with these inputs
    //h(pose, surfel)

    //With pose : R and t
    //With surfel : Snx, Sny, Snz, Sx, Sy, Sz

    Matrix Hn1;//<3, 3>
    Matrix Hn2;//<3, 3>

    Unit3 normal_in(surfel.nx, surfel.ny, surfel.nz);
    Unit3 normal_out = pose.rotation().unrotate(normal_in, Hn1, Hn2);
    
    Matrix Hx1;//<3, 6>
    Matrix Hx2;//<3, 3>

    Point3 point_in(surfel.x, surfel.y, surfel.z);
    Point3 point_out = pose.transformTo(point_in, Hx1, Hx2);
    
    Vector6 expected;
    Vector3 norm = 1.0f*normal_out;
    expected.segment(0,3) = norm;
    expected.segment(3,3) = point_out.vector();


    if(H1){
        Matrix H1_ = Matrix::Zero(6,6);
        H1_.block(0,0,3,3) = Hn1;
        H1_.block(3,0,3,6) = Hx1;
        *H1 = H1_;
    }
    if(H2){
        Matrix H2_ = Matrix::Zero(6,6);
        H2_.block(0,0,3,3) = Hn2;
        H2_.block(3,3,3,3) = Hx2;
        *H2 = H2_;
    }

    Vector6 result;
    result = expected - measured_;
    std::cout<<"results"<<std::endl;
    std::cout<<result.transpose()<<std::endl;
    return result;
    
}

gtsam::Matrix3 PlaneMeasureFactorNew::skewsym_matrix(gtsam::Vector3 & vec) const{

    gtsam::Matrix3 result = gtsam::Matrix3::Zero(3,3);
    result(0,1) = -vec(2);
    result(0,2) = vec(1);
    result(1,0) = vec(2);
    result(1,2) = -vec(0);
    result(2,0) = -vec(1);
    result(2,1) = vec(0);
    return result;
}