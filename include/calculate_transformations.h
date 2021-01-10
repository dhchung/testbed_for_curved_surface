#pragma once
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <surfel_node.h>
#include <surfel_node_new.h>

class CalTransform{
public:
    CalTransform();
    ~CalTransform();

    void xyzrpy2t(float x, float y, float z, 
                  float roll, float pitch, float yaw, 
                  Eigen::Matrix4f * T);

    void xyzrpy2t(double x, double y, double z, 
                  double roll, double pitch, double yaw, 
                  gtsam::Matrix4 * T);

    void xyzrpy2t(std::vector<float> state, Eigen::Matrix4f * T);

    Eigen::Matrix4f xyzrpy2t(float x, float y, float z, 
                             float roll, float pitch, float yaw);
    Eigen::Matrix4f xyzrpy2t(std::vector<float> state);


    void t2xyzrpy(Eigen::Matrix4f T, std::vector<float> * xyzrpy);

    void rpy2r(float roll, float pitch, float yaw, Eigen::Matrix3f * R);
    void rpy2r(double roll, double pitch, double yaw, gtsam::Matrix3 * R);

    void r2rpy(Eigen::Matrix3f R, std::vector<float> * rpy);

    void inverse_t(Eigen::Matrix4f T1, Eigen::Matrix4f *T2);
    Eigen::Matrix4f inverse_t(Eigen::Matrix4f T);

    Eigen::Vector4f transform_plane(Eigen::Matrix4f &T1, Eigen::Vector4f & p1,  Eigen::Matrix4f &T2);
    gtsam::Vector4 transform_plane(gtsam::Matrix4 &T1, gtsam::Vector4 &p1, gtsam::Matrix4 &T2);

    Eigen::Vector3f transform_point(Eigen::Matrix4f &T, Eigen::Vector3f &pt);
    gtsam::Vector3 transform_point(gtsam::Matrix4 &T, gtsam::Vector3 &pt);

    void odometry_calculation(float & prev_x, float & prev_y, float & prev_z, float & prev_roll, float & prev_pitch, float & prev_yaw,
                              float & cur_dx, float & cur_dy, float & cur_dz, float & cur_droll, float & cur_dpitch, float & cur_dyaw,
                              std::vector<float>* cur_state);
    
    surfelnode::Surfel get_initial_guess(gtsam::Pose3 & pose_initial, gtsam::Vector4 & measurement);
    surfelnode::Surfel get_initial_guess(gtsam::Pose3 & pose_initial, gtsam::Vector6 & measurement);
    
    surfelnodenew::Surfel get_initial_guess_new(gtsam::Pose3 & pose_initial, gtsam::Vector6 & measurement);

    gtsam::Pose3 dxyzrpy2Pose3(std::vector<float> &dstate);

    Eigen::Matrix4f Pose32Matrix4(gtsam::Pose3& pose);

    gtsam::Vector4 surfel2plane(gtsam::Pose3& pose, surfelnode::Surfel & surfel);

};