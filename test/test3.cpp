#include <iostream>
#include "calculate_transformations.h"
#include "opengl_rendering.h"
#include "data_load_module.h"
#include "surfel_measure_factor.h"
#include "surfel_node_new.h"
#include "parameters_json.h"
#include "coplanar_factor_new.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <jsoncpp/json/json.h>
#include <fstream>

OpenglRendering ogl_rendering("3D Point Cloud (non-sequential)");

CalTransform c_trans;
using namespace std;
using namespace gtsam;
int main(){
    Params params;
    std::string param_dir = "./include/parameters.json";
    params.read_data(param_dir);


    params.camParam.PrintStuff();
    params.SLAMParam.PrintStuff();
    std::cout<<"------------------------------"<<std::endl;


    //Testing for two surfels
    //yaw change with same plane measures
    //state = [x, y, z, roll, pitch, yaw]
    //measure = [nx, ny, nz, d]
    //surfel = [nx, ny, nz, x, y, z]

    //Initial state = [0, 0, 0, 0, 0, 0]
    //Second state = [0, 0, 0, 0, 0, pi/6]

    std::vector<float> state_0{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<float> d_state{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, M_PI/6};
    std::vector<float> d_state2{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0*M_PI/4};
    // std::vector<float> d_state2{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0};
    

    NonlinearFactorGraph graph;
    Values initials;
    Values results;

    vector<Symbol> X;
    vector<Symbol> L;

    int gtsam_idx = 0;

    noiseModel::Diagonal::shared_ptr priorNoise = 
        noiseModel::Diagonal::Sigmas((Vector(6)<<params.SLAMParam.init_noise_angle, 
                                                 params.SLAMParam.init_noise_angle, 
                                                 params.SLAMParam.init_noise_angle,
                                                 params.SLAMParam.init_noise_translation, 
                                                 params.SLAMParam.init_noise_translation, 
                                                 params.SLAMParam.init_noise_translation).finished());

    noiseModel::Diagonal::shared_ptr odomNoise = 
        noiseModel::Diagonal::Sigmas((Vector(6)<<params.SLAMParam.odom_noise_angle, 
                                                 params.SLAMParam.odom_noise_angle, 
                                                 params.SLAMParam.odom_noise_angle,
                                                 params.SLAMParam.odom_noise_translation, 
                                                 params.SLAMParam.odom_noise_translation, 
                                                 params.SLAMParam.odom_noise_translation).finished());

    // noiseModel::Diagonal::shared_ptr measNoise = 
    //     noiseModel::Diagonal::Sigmas((Vector(6)<<params.SLAMParam.measure_noise_normal, 
    //                                              params.SLAMParam.measure_noise_normal, 
    //                                              params.SLAMParam.measure_noise_normal,
    //                                              params.SLAMParam.measure_noise_distance,
    //                                              params.SLAMParam.measure_noise_distance/2.0,
    //                                              params.SLAMParam.measure_noise_distance/2.0).finished());

    noiseModel::Diagonal::shared_ptr measNoise = 
        noiseModel::Diagonal::Sigmas((Vector(5)<<params.SLAMParam.measure_noise_normal, 
                                                 params.SLAMParam.measure_noise_normal,
                                                 params.SLAMParam.measure_noise_distance,
                                                 params.SLAMParam.measure_noise_distance/2.0,
                                                 params.SLAMParam.measure_noise_distance/2.0).finished());




    Vector6 measurement;
    measurement<<-1.0, 0.0, 0.0, 1.0, 0.0, 0.0;


    X.push_back(Symbol('x', gtsam_idx));
    L.push_back(Symbol('l', gtsam_idx));
    Pose3 prior_state = c_trans.dxyzrpy2Pose3(state_0);
    graph.add(PriorFactor<Pose3>(X[gtsam_idx], prior_state, priorNoise));
    graph.add(boost::make_shared<SurfelMeasureFactor>(X[gtsam_idx], L[gtsam_idx], measurement, measNoise));
    initials.insert(X[gtsam_idx], prior_state);
    initials.insert(L[gtsam_idx], c_trans.get_initial_guess_new(prior_state, measurement));

    X.push_back(Symbol('x', gtsam_idx+1));
    L.push_back(Symbol('l', gtsam_idx+1));    
    
    Pose3 d_state_pose_true = c_trans.dxyzrpy2Pose3(d_state);
    Pose3 d_state_pose3 = c_trans.dxyzrpy2Pose3(d_state2);
    Pose3 e_state = d_state_pose3;
    Surfel e_surfel = c_trans.get_initial_guess_new(e_state, measurement);

    
    graph.add(BetweenFactor<Pose3>(X[gtsam_idx], X[gtsam_idx+1], d_state_pose_true, odomNoise));
    graph.add(boost::make_shared<SurfelMeasureFactor>(X[gtsam_idx+1], L[gtsam_idx+1], measurement, measNoise));
    initials.insert(X[gtsam_idx+1], e_state);
    initials.insert(L[gtsam_idx+1], e_surfel);


    noiseModel::Diagonal::shared_ptr coplanarNoise = 
        noiseModel::Diagonal::Sigmas((Vector(3)<<params.SLAMParam.measure_noise_normal, 
                                                 params.SLAMParam.measure_noise_normal,
                                                 params.SLAMParam.measure_noise_distance).finished());


    graph.add(boost::make_shared<CoplanarFactorNew>(L[gtsam_idx], L[gtsam_idx+1], coplanarNoise));

    // initials.insert(X[gtsam_idx+1], prior_state);
    // initials.insert(L[gtsam_idx+1], c_trans.get_initial_guess_new(prior_state, measurement));


    LevenbergMarquardtParams parameters;
    parameters.setVerbosity("Error");
    LevenbergMarquardtOptimizer optimizer(graph, initials, parameters);
    results = optimizer.optimize();
    // results = LevenbergMarquardtOptimizer(graph, initials).optimize();
    // results = GaussNewtonOptimizer(graph, initials).optimize();

    // results.print("asdf");


    std::vector<std::vector<float>> surfel_optimized;
    std::vector<std::vector<float>> surfel_initial;

    for(int i = 0; i<L.size(); ++i) {
        Surfel initial_surfel = initials.at<Surfel>(L[i]);
        Surfel optimized_surfel = results.at<Surfel>(L[i]);

        std::vector<float> surfel_initials_now{(float)initial_surfel.normal_dir.unitVector().x(),
                                               (float)initial_surfel.normal_dir.unitVector().y(),
                                               (float)initial_surfel.normal_dir.unitVector().z(),
                                               (float)initial_surfel.position.x(),
                                               (float)initial_surfel.position.y(),
                                               (float)initial_surfel.position.z()};
        surfel_initial.push_back(surfel_initials_now);

        std::vector<float> surfel_optimized_now{(float)optimized_surfel.normal_dir.unitVector().x(),
                                                (float)optimized_surfel.normal_dir.unitVector().y(),
                                                (float)optimized_surfel.normal_dir.unitVector().z(),
                                                (float)optimized_surfel.position.x(),
                                                (float)optimized_surfel.position.y(),
                                                (float)optimized_surfel.position.z()};
        surfel_optimized.push_back(surfel_optimized_now);

        // std::cout<<"Initial Surfel at "<<i<<std::endl;
        // initial_surfel.print_surfel();
        // std::cout<<"Optimized Surfel at "<<i<<std::endl;
        // optimized_surfel.print_surfel();

    }

    std::vector<float> color0{1.0f, 1.0f, 0.0f};
    std::vector<float> color1{1.0f, 0.0f, 0.0f};

    Eigen::Matrix4f initialize_matrix = Eigen::Matrix4f::Zero(4,4);

    std::vector<Eigen::Matrix4f> state_optimized(X.size(), initialize_matrix);
    std::vector<Eigen::Matrix4f> state_initial;
    state_initial.resize(X.size());
    

    for(int j = 0; j<X.size(); ++j) {
        Pose3 optimized_state = results.at<Pose3>(X[j]);
        Pose3 initial_state2 = initials.at<Pose3>(X[j]);


        state_optimized[j].block(0,0,4,4) = c_trans.Pose32Matrix4(optimized_state);
        state_initial[j].block(0,0,4,4) = c_trans.Pose32Matrix4(initial_state2);

        // std::cout<<"Initials at state "<<j<<std::endl;
        // std::cout<<state_initial[j]<<std::endl;
        // std::cout<<"Finals at state "<<j<<std::endl;
        // std::cout<<state_optimized[j]<<std::endl;

    }

    ogl_rendering.init_opengl();
    ogl_rendering.draw_surfels_init_n_final(state_optimized,
                                            state_initial,
                                            surfel_optimized, 
                                            surfel_initial, 
                                            color0, 
                                            color1,
                                            params);
    ogl_rendering.terminate();

    return 0;
}