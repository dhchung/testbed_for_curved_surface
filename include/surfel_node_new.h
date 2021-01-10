#pragma once
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <iostream>

namespace surfelnodenew{
    struct Surfel{
        gtsam::Unit3 normal_dir;
        gtsam::Point3 position;


        Surfel(double nxi, double nyi, double nzi,
               double xi, double yi, double zi):
               normal_dir(gtsam::Unit3(nxi, nyi, nzi)),
               position(gtsam::Point3(xi, yi, zi)){}
        Surfel(gtsam::Unit3 s_n, gtsam::Point3 s_p):
               normal_dir(s_n), position(s_p){} 
        Surfel(){}    
    };
}

namespace gtsam{
    template<>
    struct traits<surfelnodenew::Surfel>{
        static void Print(const surfelnodenew::Surfel & m, const std::string & str = ""){
            std::cout<<str<<std::endl;
            std::cout<<"normal_direction"<<std::endl;
            std::cout<<m.normal_dir<<std::endl;
            std::cout<<"position"<<std::endl;
            std::cout<<m.position<<std::endl;
        }

        static bool Equals(const surfelnodenew::Surfel &m1, const surfelnodenew::Surfel &m2,
                           double tol = 1e-8){
            double theta = acos(m1.normal_dir.unitVector().transpose()*m2.normal_dir.unitVector());
            Vector3 delta = m1.position - m2.position;
            double distance = sqrt(delta.transpose()*delta);
        if((std::abs(theta)<tol || std::abs(std::abs(theta) - M_PI) < tol) 
                && distance < tol) {
                return true;
            } else {
                return false;
            }
        }

        enum{dimension = 5};
        static int GetDimension(const surfelnodenew::Surfel&) {return dimension;}
        
        typedef surfelnodenew::Surfel ManifoldType;
        typedef Eigen::Matrix<double, dimension, 1> TangentVector;

        static TangentVector Local(const surfelnodenew::Surfel& origin,
                                   const surfelnodenew::Surfel& other) {
            Vector5 result;
            Vector2 normal_local = origin.normal_dir.localCoordinates(other.normal_dir);
            Vector3 position_local = origin.position.localCoordinates(other.position);
            result << normal_local, position_local;
        }

        static surfelnodenew::Surfel Retract(const surfelnodenew::Surfel & origin,
                                          const TangentVector& v) {
            Matrix22 H_n;
            Vector2 v_n(v(0), v(1));
            Vector3 v_p(v(2), v(3), v(4));
            Unit3 normal_retract = origin.normal_dir.retract(v_n, H_n);
            Point3 position_retract = origin.position.retract(v_p);
            return surfelnodenew::Surfel(normal_retract, position_retract);
        }
    };
}