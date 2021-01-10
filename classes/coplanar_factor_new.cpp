#include "coplanar_factor_new.h"

CoplanarFactorNew::CoplanarFactorNew(Key key1, Key key2, 
                                     const SharedNoiseModel& model):
                                     NoiseModelFactor2(model, key1, key2) {}

Vector CoplanarFactorNew::evaluateError(const Surfel& surfel1, const Surfel & surfel2,
                                        boost::optional<Matrix&> H1,
                                        boost::optional<Matrix&> H2) const{

    //With surfels expressed in global coordinate system

    Unit3 sn1 = surfel1.normal_dir;
    Point3 st1 = surfel1.position;
    Unit3 sn2 = surfel2.normal_dir;
    Point3 st2 = surfel2.position;


    Matrix He1;
    Matrix He2;

    Vector2 error_normal = sn2.errorVector(sn1, He2, He1);
    Vector1 error_position = -sn2.unitVector().transpose()*st2.vector() + sn1.unitVector().transpose()*st1.vector();

    if(H1){
        Matrix H1_ = Matrix::Zero(3,5);
        H1_.block(0,0,2,2) = He1;
        H1_.block(2,0,1,2) = st1.transpose()*sn1.basis();
        H1_.block(2,2,1,3) = sn1.unitVector().transpose();
        *H1 = H1_;
    }
    if(H2){
        Matrix H2_ = Matrix::Zero(3,5);
        H2_.block(0,0,2,2) = He2;
        H2_.block(2,0,1,3) = -st2.transpose()*sn2.basis();
        H2_.block(2,2,1,3) = -sn1.unitVector().transpose();
        *H2 = H2_;
    }

    Vector3 result;
    result<<error_normal, error_position;

    return result;
}
