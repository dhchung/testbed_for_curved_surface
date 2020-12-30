#include "coplanar_factor.h"

CoplanarFactor::CoplanarFactor(Key key1, Key key2, 
                               const SharedNoiseModel& model):
                               NoiseModelFactor2(model, key1, key2) {}

Vector CoplanarFactor::evaluateError(const Surfel& surfel1, const Surfel & surfel2,
                                     boost::optional<Matrix&> H1,
                                     boost::optional<Matrix&> H2) const{

    //With surfels expressed in global coordinate system

    if(H1){
        Matrix H1_ = Matrix::Zero(4,6);
        H1_.block(0,0,3,3) = -Matrix::Identity(3,3);
        H1_(3,0) = surfel1.x;
        H1_(3,1) = surfel1.y;
        H1_(3,2) = surfel1.z;
        H1_(3,3) = surfel1.nx;
        H1_(3,4) = surfel1.ny;
        H1_(3,5) = surfel1.nz;
        *H1 = H1_;
    }
    if(H2){
        Matrix H2_ = Matrix::Zero(4,6);
        H2_.block(0,0,3,3) = -Matrix::Identity(3,3);
        H2_(3,0) = -surfel2.x;
        H2_(3,1) = -surfel2.y;
        H2_(3,2) = -surfel2.z;
        H2_(3,3) = -surfel2.nx;
        H2_(3,4) = -surfel2.ny;
        H2_(3,5) = -surfel2.nz;
        *H2 = H2_;
    }

    Vector4 result;
    result(0) = surfel2.nx - surfel1.nx;
    result(1) = surfel2.ny - surfel1.ny;
    result(2) = surfel2.nz - surfel1.nz;
    result(3) = -(surfel2.nx*surfel2.x + surfel2.ny*surfel2.y + surfel2.nz*surfel2.z) +
                (surfel1.nx*surfel1.x + surfel1.ny*surfel1.y + surfel1.nz*surfel1.z);


    return result;
}
