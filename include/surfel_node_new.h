#pragma once
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <iostream>

namespace surfelnode{
    struct Surfel{
        gtsam::Unit3 normal_dir;
        gtsam::Point3 position;


        Surfel(double nxi, double nyi, double nzi,
               double xi, double yi, double zi):
               normal_dir(gtsam::Unit3(nxi, nyi, nzi)),
               position(gtsam::Point3(xi, yi, zi)){}
        Surfel(){}    
    }
};

namespace gtsam{
    template<>
    struct traits<surfelnode::Surfel>{
        static void Print(const surfelnode::Surfel & m, const std::string & str = ""){
            asdasdas
        }
    }
}