#include "surfel_measure_factor_plane.h"

SurfelMeasureFactorPlane::SurfelMeasureFactorPlane(Key key1, Key key2, 
                                       const Vector4& measured, const SharedNoiseModel& model):
                                       NoiseModelFactor2(model, key1, key2),
                                       measured_(measured){}

Vector SurfelMeasureFactorPlane::evaluateError(const Pose3& pose, const Surfel & surfel,
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
    double m_d(measured_(3));
    // Point3 m_p(measured_(3), measured_(4), measured_(5));


    Matrix HR;  //2*3
    Matrix Hp;  //2*2
    Unit3 cur_sn = pose.rotation().unrotate(g_sn, HR, Hp);

    Matrix Dpose; //3*6
    Matrix Dpoint; //3*3
    Point3 cur_st = pose.transformTo(g_st, Dpose, Dpoint);
    double cur_d = cur_sn.unitVector().transpose()*(pose.translation().vector() - g_st);

    // std::cout<<"HR"<<std::endl;
    // std::cout<<HR<<std::endl;


    Matrix Hep; // 2*2
    Matrix Heq; // 2*2

    Vector2 error_normal = m_n.errorVector(cur_sn, Heq, Hep);
    // Vector3 error_position = m_p.localCoordinates(cur_st);
    double error_d = cur_d - m_d;

    if(H1) {
        Matrix H1_ = Matrix::Zero(3,6);
        H1_.block(0,0,2,3) = HR;
        H1_.block(2,3,1,3) = g_sn.unitVector().transpose();
        *H1 = H1_;
    }
    if(H2) {
        Matrix H2_ = Matrix::Zero(3,5);
        H2_.block(0,0,2,2) = Hep*Hp;
        H2_.block(2,0,1,2) = (g_st - pose.translation().vector()).transpose()*g_sn.basis();
        H2_.block(2,2,1,3) = g_sn.unitVector().transpose();
        *H2 = H2_;
    }



    Vector5 result;

    result << error_normal, error_d;
    return result;

}
