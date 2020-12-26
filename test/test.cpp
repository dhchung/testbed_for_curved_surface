#include <iostream>
#include "calculate_transformations.h"
#include "opengl_rendering.h"
#include "data_load_module.h"
#include "plane_measure_factor.h"
#include "surfel_node.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

OpenglRendering ogl_rendering("3D Point Cloud (non-sequential)");

using namespace std;
using namespace gtsam;
int main(){

    //Testing for two surfels
    //yaw change with same plane measures
    //state = [x, y, z, roll, pitch, yaw]
    //measure = [nx, ny, nz, d]
    //surfel = [nx, ny, nz, x, y, z]

    //Initial state = [0, 0, 0, 0, 0, 0]
    //Second state = [0, 0, 0, 0, 0, pi/6]


    NonlinearFactorGraph graph;
    Values initials;
    Values results;

    vector<Symbol> X;
    vector<Symbol> L;

    int gtsam_idx = 0;


    vector<vector<float>> surfel_test;

    ogl_rendering.init_opengl();

    vector<float> s1;
    s1.resize(6);
    s1[0] = -1.0f;
    s1[1] = 0.0f;
    s1[2] = 0.0f;
    s1[3] = 1.0f;
    s1[4] = 0.0f;
    s1[5] = 0.0f;

    ogl_rendering.draw_surfels(surfel_test);

    ogl_rendering.terminate();

    return 0;
}