#pragma once

#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <string>
#include <math.h>

struct CameraParameters{
    float base_line = 0.0f;
    float img_center_x = 0.0f;
    float img_center_y = 0.0f;
    float focal_length = 0.0f;
    float depth_err = 0.0f;

    void PrintStuff(){
        std::cout<<"Camera Parmeters"<<std::endl;
        std::cout<<"base_line: "<<base_line<<std::endl;
        std::cout<<"img_center_x: "<<img_center_x<<std::endl;
        std::cout<<"img_center_y: "<<img_center_y<<std::endl;
        std::cout<<"focal_length: "<<focal_length<<std::endl;
        std::cout<<"depth_err: "<<depth_err<<std::endl;
    }

};
struct SLAMParameters{
    float odom_noise_translation = 0.0f;
    float odom_noise_angle = 0.0f;
    float init_noise_translation = 0.0f;
    float init_noise_angle = 0.0f;
    float measure_noise_normal = 0.0f;
    float measure_noise_distance = 0.0f;
    float absolute_noise_translation = 0.0f;
    float absolute_noise_angle = 0.0f;
    void PrintStuff(){
        std::cout<<"SLAM Parmeters"<<std::endl;
        std::cout<<"odom_noise_translation: "<<odom_noise_translation<<std::endl;
        std::cout<<"odom_noise_angle: "<<odom_noise_angle<<std::endl;
        std::cout<<"init_noise_translation: "<<init_noise_translation<<std::endl;
        std::cout<<"init_noise_angle: "<<init_noise_angle<<std::endl;
        std::cout<<"measure_noise_normal: "<<measure_noise_normal<<std::endl;
        std::cout<<"measure_noise_distance: "<<measure_noise_distance<<std::endl;
        std::cout<<"absolute_noise_translation: "<<absolute_noise_translation<<std::endl;
        std::cout<<"absolute_noise_angle: "<<absolute_noise_angle<<std::endl;
    }

};


class Params{
public:
    Params();
    ~Params();
    void read_data(std::string & json_dir);
    float value2float(Json::ValueIterator & it);

    CameraParameters camParam;
    SLAMParameters SLAMParam;
};