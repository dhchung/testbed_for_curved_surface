#pragma once
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <iostream>

namespace surfelnode{
    struct Surfel{
        double nx;
        double ny;
        double nz;
        double x;
        double y;
        double z;

    Surfel(double nxi, double nyi, double nzi, 
               double xi, double yi, double zi):
               nx(nxi), ny(nyi), nz(nzi),
               x(xi), y(yi), z(zi){}
    Surfel(){}

    void print_surfel(){
        std::cout<<"nx: "<<nx<<std::endl;
        std::cout<<"ny: "<<ny<<std::endl;
        std::cout<<"nz: "<<nz<<std::endl;
        std::cout<<"x: "<<x<<std::endl;
        std::cout<<"y: "<<y<<std::endl;
        std::cout<<"z: "<<z<<std::endl;
    }

    };
}

namespace gtsam{
    template<>
    struct traits<surfelnode::Surfel>{

        static float BetweenAngle(const surfelnode::Surfel & m1, const surfelnode::Surfel & m2){
            float angle;
            angle = acos(m1.nx*m2.nx + m1.ny*m2.ny + m1.nz*m2.nz);
            return angle;
        }

        static void Print(const surfelnode::Surfel & m, const std::string & str = ""){
            std::cout<<str<<"("<<m.nx<<", "<<m.ny<<", "<<m.nz<<", "
                     <<m.x<<", "<<m.y<<", "<<m.z<<")"<<std::endl;
        }

        static bool Equals(const surfelnode::Surfel &m1, const surfelnode::Surfel &m2, double tol = 1e-8){
            if(fabs(m1.x-m2.x)<tol &&
            fabs(m1.y-m2.y)<tol &&
            fabs(m1.z-m2.z)<tol &&
            BetweenAngle(m1, m2)<tol){
                return true;
            }else{
                    return false;
            }
        }

        enum{dimension = 6};
        static int GetDimension(const surfelnode::Surfel&) {return dimension;}

        typedef surfelnode::Surfel ManifoldType;
        typedef Eigen::Matrix<double, dimension, 1> TangentVector;

        static TangentVector Local(const surfelnode::Surfel& origin,
                                   const surfelnode::Surfel& other){

            Vector6 result;
            result(0) = other.nx-origin.nx;
            result(1) = other.ny-origin.ny;
            result(2) = other.nz-origin.nz;
            result(3) = other.x-origin.x;
            result(4) = other.y-origin.y;
            result(5) = other.z-origin.z;

            return result;
        }

        static surfelnode::Surfel Retract(const surfelnode::Surfel& origin,
                                                    const TangentVector& v){
            return surfelnode::Surfel(origin.x+v(0),
                                      origin.y+v(1),
                                      origin.z+v(2),
                                      origin.nx+v(3),
                                      origin.ny+v(4),
                                      origin.nz+v(5));
        }

    };
}

