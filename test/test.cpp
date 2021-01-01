#include <iostream>
#include "calculate_transformations.h"
#include "opengl_rendering.h"
#include "data_load_module.h"
#include "plane_measure_factor.h"
#include "surfel_node.h"
#include "parameters.h"
#include "coplanar_factor.h"

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

CalTransform c_trans;
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

    std::vector<float> state_0{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<float> d_state{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, M_PI/6};
    // std::vector<float> d_state2{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, M_PI/5};
    std::vector<float> d_state2{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0};
    

    NonlinearFactorGraph graph;
    Values initials;
    Values results;

    vector<Symbol> X;
    vector<Symbol> L;

    int gtsam_idx = 0;

    noiseModel::Diagonal::shared_ptr priorNoise = 
        noiseModel::Diagonal::Sigmas((Vector(6)<<init_noise_angle, 
                                                 init_noise_angle, 
                                                 init_noise_angle,
                                                 init_noise_translation, 
                                                 init_noise_translation, 
                                                 init_noise_translation).finished());

    noiseModel::Diagonal::shared_ptr odomNoise = 
        noiseModel::Diagonal::Sigmas((Vector(6)<<odom_noise_angle, 
                                                 odom_noise_angle, 
                                                 odom_noise_angle,
                                                 odom_noise_translation, 
                                                 odom_noise_translation, 
                                                 odom_noise_translation).finished());

    // noiseModel::Diagonal::shared_ptr measNoise = 
    //     noiseModel::Diagonal::Sigmas((Vector(6)<<measure_noise_normal, 
    //                                              measure_noise_normal, 
    //                                              measure_noise_normal,
    //                                              measure_noise_distance, 
    //                                              measure_noise_distance, 
    //                                              measure_noise_distance).finished());

    noiseModel::Diagonal::shared_ptr measNoise = 
        noiseModel::Diagonal::Sigmas((Vector(4)<<measure_noise_normal, 
                                                 measure_noise_normal, 
                                                 measure_noise_normal,
                                                 measure_noise_distance).finished());


    Vector4 measurement;
    measurement<<-1.0, 0.0, 0.0, 1.0;

    X.push_back(Symbol('x', gtsam_idx));
    L.push_back(Symbol('l', gtsam_idx));
    Pose3 prior_state = c_trans.dxyzrpy2Pose3(state_0);
    graph.add(PriorFactor<Pose3>(X[gtsam_idx], prior_state, priorNoise));
    graph.add(boost::make_shared<PlaneMeasureFactor>(X[gtsam_idx], L[gtsam_idx], measurement, measNoise));
    initials.insert(X[gtsam_idx], prior_state);
    initials.insert(L[gtsam_idx], c_trans.get_initial_guess(prior_state, measurement));

    X.push_back(Symbol('x', gtsam_idx+1));
    L.push_back(Symbol('l', gtsam_idx+1));    
    
    Pose3 d_state_pose_true = c_trans.dxyzrpy2Pose3(d_state);
    Pose3 d_state_pose3 = c_trans.dxyzrpy2Pose3(d_state2);
    Pose3 e_state = d_state_pose3;
    Surfel e_surfel = c_trans.get_initial_guess(e_state, measurement);

    
    graph.add(BetweenFactor<Pose3>(X[gtsam_idx], X[gtsam_idx+1], d_state_pose_true, odomNoise));
    graph.add(boost::make_shared<PlaneMeasureFactor>(X[gtsam_idx+1], L[gtsam_idx+1], measurement, measNoise));
    initials.insert(X[gtsam_idx+1], e_state);
    initials.insert(L[gtsam_idx+1], e_surfel);

    // initials.insert(X[gtsam_idx+1], prior_state);
    // initials.insert(L[gtsam_idx+1], c_trans.get_initial_guess(prior_state, measurement));



    double meas_noise_n = measure_noise_normal/200;
    double meas_noise_d = measure_noise_distance/200;    

    gtsam::noiseModel::Diagonal::shared_ptr measNoise_d = 
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(4)<<meas_noise_n,
                                                                meas_noise_n,
                                                                meas_noise_n,
                                                                meas_noise_d).finished());

    // graph.add(boost::make_shared<CoplanarFactor>(L[gtsam_idx], L[gtsam_idx+1], measNoise_d)); 


    results = LevenbergMarquardtOptimizer(graph, initials).optimize();


    std::vector<std::vector<float>> surfel_optimized;
    std::vector<std::vector<float>> surfel_initial;

    for(int i = 0; i<L.size(); ++i) {
        Surfel initial_surfel = initials.at<Surfel>(L[i]);
        Surfel optimized_surfel = results.at<Surfel>(L[i]);

        std::vector<float> surfel_initials_now{(float)initial_surfel.nx,
                                               (float)initial_surfel.ny,
                                               (float)initial_surfel.nz,
                                               (float)initial_surfel.x,
                                               (float)initial_surfel.y,
                                               (float)initial_surfel.z};
        surfel_initial.push_back(surfel_initials_now);

        std::vector<float> surfel_optimized_now{(float)optimized_surfel.nx,
                                                (float)optimized_surfel.ny,
                                                (float)optimized_surfel.nz,
                                                (float)optimized_surfel.x,
                                                (float)optimized_surfel.y,
                                                (float)optimized_surfel.z};
        surfel_optimized.push_back(surfel_optimized_now);

        std::cout<<"Initial Surfel at "<<i<<std::endl;
        initial_surfel.print_surfel();
        std::cout<<"Optimized Surfel at "<<i<<std::endl;
        optimized_surfel.print_surfel();

    }

    std::vector<float> color0{1.0f, 1.0f, 0.0f};
    std::vector<float> color1{1.0f, 0.0f, 0.0f};

    std::vector<Eigen::Matrix4f> state_optimized;
    state_optimized.resize(X.size());
    std::vector<Eigen::Matrix4f> state_initial;
    state_initial.resize(X.size());
    

    for(int j = 0; j<X.size(); ++j) {
        Pose3 optimized_state = results.at<Pose3>(X[j]);
        Pose3 initial_state2 = initials.at<Pose3>(X[j]);
        state_optimized[j] = c_trans.Pose32Matrix4(optimized_state);
        state_initial[j] = c_trans.Pose32Matrix4(initial_state2);

        std::cout<<"Initials at state "<<j<<std::endl;
        std::cout<<state_initial[j]<<std::endl;
        std::cout<<"Finals at state "<<j<<std::endl;
        std::cout<<state_optimized[j]<<std::endl;
    }

    ogl_rendering.init_opengl();
    ogl_rendering.draw_surfels_init_n_final(state_optimized,
                                            state_initial,
                                            surfel_optimized, 
                                            surfel_initial, 
                                            color0, 
                                            color1);
    ogl_rendering.terminate();

    return 0;
}