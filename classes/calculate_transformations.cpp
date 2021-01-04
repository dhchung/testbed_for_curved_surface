#include "calculate_transformations.h"

CalTransform::CalTransform(){
}

CalTransform::~CalTransform(){

}

void CalTransform::xyzrpy2t(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Matrix4f * T){

    Eigen::Matrix3f R;
    rpy2r(roll, pitch, yaw, &R);
    Eigen::Vector3f trans;
    trans << x, y, z;

    T->setZero(4,4);
    T->block(0,0,3,3) = R;
    T->block(0,3,3,1) = trans;
    T->operator()(3,3) = 1.0f;

}

void CalTransform::xyzrpy2t(double x, double y, double z, 
                            double roll, double pitch, double yaw, 
                            gtsam::Matrix4 * T){

    gtsam::Matrix3 R;
    rpy2r(roll, pitch, yaw, &R);
    gtsam::Vector3 trans;
    trans << x, y, z;

    T->setZero(4,4);
    T->block(0,0,3,3) = R;
    T->block(0,3,3,1) = trans;
    T->operator()(3,3) = 1.0f;

}


void CalTransform::xyzrpy2t(std::vector<float> state, Eigen::Matrix4f * T){

    float x = state[0];
    float y = state[1];
    float z = state[2];
    float roll = state[3];
    float pitch = state[4];
    float yaw = state[5];
    
    Eigen::Matrix3f R;
    rpy2r(roll, pitch, yaw, &R);
    Eigen::Vector3f trans;
    trans << x, y, z;

    T->setZero(4,4);
    T->block(0,0,3,3) = R;
    T->block(0,3,3,1) = trans;
    T->operator()(3,3) = 1.0f;

}


Eigen::Matrix4f CalTransform::xyzrpy2t(float x, float y, float z, float roll, float pitch, float yaw){

    Eigen::Matrix3f R;
    rpy2r(roll, pitch, yaw, &R);
    Eigen::Vector3f trans;
    trans << x, y, z;

    Eigen::Matrix4f T;

    T.setZero(4,4);
    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = trans;
    T(3,3) = 1.0f;

    return T;
}



Eigen::Matrix4f CalTransform::xyzrpy2t(std::vector<float> state){

    float x = state[0];
    float y = state[1];
    float z = state[2];
    float roll = state[3];
    float pitch = state[4];
    float yaw = state[5];
    
    Eigen::Matrix3f R;
    rpy2r(roll, pitch, yaw, &R);
    Eigen::Vector3f trans;
    trans << x, y, z;

    Eigen::Matrix4f T;

    T.setZero(4,4);
    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = trans;
    T(3,3) = 1.0f;
    return T;
}





void CalTransform::t2xyzrpy(Eigen::Matrix4f T, std::vector<float> * xyzrpy){

    Eigen::Matrix3f R = T.block(0,0,3,3);
    std::vector<float> rpy;
    r2rpy(R, &rpy);
    Eigen::Vector3f xyz = T.block(0,3,3,1);

    xyzrpy->clear();
    xyzrpy->push_back(xyz(0));
    xyzrpy->push_back(xyz(1));
    xyzrpy->push_back(xyz(2));
    xyzrpy->insert(xyzrpy->end(), rpy.begin(), rpy.end());

}

void CalTransform::rpy2r(float roll, float pitch, float yaw, Eigen::Matrix3f * R){

    Eigen::Matrix3f R_roll;
    R_roll << 1.0f, 0.0f, 0.0f,
              0.0f, cos(roll), -sin(roll),
              0.0f, sin(roll), cos(roll);

    Eigen::Matrix3f R_pitch;
    R_pitch << cos(pitch), 0.0f, sin(pitch),
               0.0f, 1.0f, 0.0f,
               -sin(pitch), 0.0f, cos(pitch);
   
    Eigen::Matrix3f R_yaw;
    R_yaw << cos(yaw), -sin(yaw), 0.0f,
             sin(yaw), cos(yaw), 0.0f,
             0.0f, 0.0f, 1.0f;

    R->operator=(R_yaw*R_pitch*R_roll);

}


void CalTransform::rpy2r(double roll, double pitch, double yaw, gtsam::Matrix3 * R){

    gtsam::Matrix3 R_roll;
    R_roll << 1.0f, 0.0f, 0.0f,
              0.0f, cos(roll), -sin(roll),
              0.0f, sin(roll), cos(roll);

    gtsam::Matrix3 R_pitch;
    R_pitch << cos(pitch), 0.0f, sin(pitch),
               0.0f, 1.0f, 0.0f,
               -sin(pitch), 0.0f, cos(pitch);
   
    gtsam::Matrix3 R_yaw;
    R_yaw << cos(yaw), -sin(yaw), 0.0f,
             sin(yaw), cos(yaw), 0.0f,
             0.0f, 0.0f, 1.0f;

    R->operator=(R_yaw*R_pitch*R_roll);

}

void CalTransform::r2rpy(Eigen::Matrix3f R, std::vector<float> * rpy){

    float roll;
    float pitch;
    float yaw;


    if(R(3,1) != 1.0f || R(3,1) != -1.0f) {
        pitch = -asin(R(2,0));
        roll = atan2(R(2,1)/cos(pitch), R(2,2)/cos(pitch));
        yaw = atan2(R(1,0)/cos(pitch), R(0,0)/cos(pitch));
    } else {
        yaw = 0.0f;
        if(R(2,0)==-1){
            pitch = M_PI/2.0f;
            roll = yaw + atan2(R(0,1), R(0,2));
        } else {
            pitch = -M_PI/2.0f;
            roll = -yaw + atan2(-R(0,1), -R(0,2));
        }
    }

    rpy->push_back(roll);
    rpy->push_back(pitch);
    rpy->push_back(yaw);
}

void CalTransform::inverse_t(Eigen::Matrix4f T1, Eigen::Matrix4f * T2){
    T2->setZero(4,4);
    Eigen::Matrix3f R = T1.block(0,0,3,3);
    Eigen::Vector3f trans = T1.block(0,3,3,1);

    T2->block(0,0,3,3) = R.transpose();
    T2->block(0,3,3,1) = -R.transpose()*trans;
    T2->operator()(3,3) = 1.0f;
}

Eigen::Matrix4f CalTransform::inverse_t(Eigen::Matrix4f T){
    Eigen::Matrix3f R = T.block(0,0,3,3);
    Eigen::Vector3f trans = T.block(0,3,3,1);

    Eigen::Matrix4f result = Eigen::Matrix4f::Zero(4,4);

    result.block(0,0,3,3) = R.transpose();
    result.block(0,3,3,1) = -R.transpose()*trans;
    result(3,3) = 1.0f;
    return result;
}

Eigen::Vector4f CalTransform::transform_plane(Eigen::Matrix4f & T1, Eigen::Vector4f & p1, Eigen::Matrix4f & T2){
    Eigen::Vector3f n1 = p1.segment(0,3);
    float d1 = p1(3);

    Eigen::Matrix3f R1 = T1.block(0,0,3,3);
    Eigen::Vector3f t1 = T1.block(0,3,3,1);

    Eigen::Matrix3f R2 = T1.block(0,0,3,3);
    Eigen::Vector3f t2 = T1.block(0,3,3,1);

    Eigen::Vector3f n2 = R2.transpose()*R1*n1;
    float d2 = d1 + (t2-t1).transpose()*R1*n1;

    Eigen::Vector4f result;
    result<<n2, d2;
    return result;
}




Eigen::Vector3f CalTransform::transform_point(Eigen::Matrix4f &T, Eigen::Vector3f &pt){
    Eigen::Vector4f pt_1;
    pt_1<<pt, 1.0;
    Eigen::Vector3f result;
    Eigen::Vector4f pt_2;

    pt_2 = T*pt_1;
    result = pt_2.segment(0,3);
    return result;
}

void CalTransform::odometry_calculation(float & prev_x, float & prev_y, float & prev_z, float & prev_roll, float & prev_pitch, float & prev_yaw,
                                        float & cur_dx, float & cur_dy, float & cur_dz, float & cur_droll, float & cur_dpitch, float & cur_dyaw,
                                        std::vector<float>* cur_state){
    
}

gtsam::Vector4 CalTransform::transform_plane(gtsam::Matrix4 &T1, gtsam::Vector4 &p1, gtsam::Matrix4 &T2){

    gtsam::Vector3 n1 = p1.segment(0,3);

    double d1 = p1(3);

    gtsam::Matrix3 R1 = T1.block(0,0,3,3);
    gtsam::Vector3 t1 = T1.block(0,3,3,1);

    gtsam::Matrix3 R2 = T2.block(0,0,3,3);
    gtsam::Vector3 t2 = T2.block(0,3,3,1);

    gtsam::Vector3 n2 = R2.transpose()*R1*n1;
    double d2 = d1 + (t2-t1).transpose()*R1*n1;

    gtsam::Vector4 result;
    result<<n2, d2;
    return result;
}

gtsam::Vector3 CalTransform::transform_point(gtsam::Matrix4 &T, gtsam::Vector3 &pt){
    gtsam::Vector4 pt_1;
    pt_1<<pt, 1.0;
    gtsam::Vector3 result;
    gtsam::Vector4 pt_2;

    pt_2 = T*pt_1;
    result = pt_2.segment(0,3);
    return result;
}

surfelnode::Surfel CalTransform::get_initial_guess(gtsam::Pose3 & pose_initial, gtsam::Vector4 & measurement){
    gtsam::Matrix4 I = gtsam::Matrix4::Identity(4,4);
    gtsam::Matrix4 T = gtsam::Matrix4::Identity(4,4);
    T.block(0,0,3,3) = pose_initial.rotation().matrix();
    T.block(0,3,3,1) = pose_initial.translation().matrix();
    gtsam::Vector3 point(-measurement(3)/measurement(0), 0, 0);
    gtsam::Vector4 t_plane = transform_plane(T, measurement, I);
    gtsam::Vector3 t_point = transform_point(T, point);

    return surfelnode::Surfel(t_plane(0), t_plane(1), t_plane(2), t_point(0), t_point(1), t_point(2));
}

surfelnode::Surfel CalTransform::get_initial_guess(gtsam::Pose3 & pose_initial, gtsam::Vector6 & measurement){
    gtsam::Matrix4 I = gtsam::Matrix4::Identity(4,4);
    gtsam::Matrix4 T = gtsam::Matrix4::Identity(4,4);
    T.block(0,0,3,3) = pose_initial.rotation().matrix();
    T.block(0,3,3,1) = pose_initial.translation().matrix();
    gtsam::Vector3 point = measurement.segment(3,3);
    gtsam::Vector3 t_normal = T.block(0,0,3,3)*measurement.segment(0,3);
    gtsam::Vector3 t_point = transform_point(T, point);

    return surfelnode::Surfel(t_normal(0), t_normal(1), t_normal(2), t_point(0), t_point(1), t_point(2));
}



gtsam::Pose3 CalTransform::dxyzrpy2Pose3(std::vector<float> &dstate){
    double dx =     (double) dstate[0];
    double dy =     (double) dstate[1];
    double dz =     (double) dstate[2];
    double droll =  (double) dstate[3];
    double dpitch = (double) dstate[4];
    double dyaw =   (double) dstate[5];

    gtsam::Matrix3 R;
    rpy2r(droll, dpitch, dyaw, &R);
    gtsam::Rot3 RR = gtsam::Rot3(R);
    gtsam::Point3 dt(dx, dy, dz);
    return gtsam::Pose3(RR, dt);
}

Eigen::Matrix4f CalTransform::Pose32Matrix4(gtsam::Pose3& pose){
    Eigen::Matrix4f result = Eigen::Matrix4f::Zero(4,4);
    gtsam::Matrix3 rot = pose.rotation().matrix();
    gtsam::Vector3 trans = pose.translation().matrix();

    result(0,0) = (float)rot(0,0);
    result(0,1) = (float)rot(0,1);
    result(0,2) = (float)rot(0,2);
    result(1,0) = (float)rot(1,0);
    result(1,1) = (float)rot(1,1);
    result(1,2) = (float)rot(1,2);
    result(2,0) = (float)rot(2,0);
    result(2,1) = (float)rot(2,1);
    result(2,2) = (float)rot(2,2);
    
    result(0,3) = trans(0);
    result(1,3) = trans(1);
    result(2,3) = trans(2);
    result(3,3) = 1.0f;
    return result;
}


gtsam::Vector4 CalTransform::surfel2plane(gtsam::Pose3& pose, surfelnode::Surfel & surfel){
    gtsam::Vector4 g_plane;
    g_plane << surfel.nx, surfel.ny, surfel.nz, 
               -(surfel.nx*surfel.x + surfel.ny*surfel.y + surfel.nz*surfel.z);
    gtsam::Matrix4 I = gtsam::Matrix4::Identity(4, 4);
    gtsam::Matrix4 T = gtsam::Matrix4::Identity(4,4);
    T.block(0,0,3,3) = pose.rotation().matrix();
    T.block(0,3,3,1) = pose.translation().matrix();
    return transform_plane(I, g_plane, T);
}
