#include "parameters_json.h"

Params::Params(){

}

Params::~Params(){

}


void Params::read_data(std::string & json_dir) {
    std::ifstream stream;
    stream.open(json_dir);

    Json::Value root;
    stream >> root;

    Json::Value camera_parms = root["camera_params"];
    Json::ValueIterator it = camera_parms.begin();

    while(it != camera_parms.end()) {
        if(it->isObject()) {
            std::string name = (*it)["name"].asString();
            if(name == "base_line") {
                camParam.base_line = value2float(it);
            }else if(name=="img_center_x") {
                camParam.img_center_x = value2float(it);
            }else if(name=="img_center_y") {
                camParam.img_center_y = value2float(it);
            }else if(name == "focal_length") {
                camParam.focal_length = value2float(it);
            }else if(name == "depth_err") {
                camParam.depth_err = value2float(it);
            }
        }
        ++it;
    }

    Json::Value slam_parms = root["slam_params"];
    it = slam_parms.begin();

    while(it != slam_parms.end()) {
        if(it->isObject()) {
            std::string name = (*it)["name"].asString();
            if(name == "odom_noise_translation") {
                SLAMParam.odom_noise_translation = value2float(it);
            }else if(name=="odom_noise_angle") {
                SLAMParam.odom_noise_angle = value2float(it)*M_PI/180.0f;
                
            }else if(name=="init_noise_translation") {
                SLAMParam.init_noise_translation = value2float(it);
            }else if(name == "init_noise_angle") {
                SLAMParam.init_noise_angle = value2float(it)*M_PI/180.0f;

            }else if(name == "measure_noise_normal") {
                SLAMParam.measure_noise_normal = value2float(it);
            }else if(name == "measure_noise_distance") {
                SLAMParam.measure_noise_distance = value2float(it);

            }else if(name == "absolute_noise_translation") {
                SLAMParam.absolute_noise_translation = value2float(it);
            }else if(name == "absolute_noise_angle") {
                SLAMParam.absolute_noise_angle = value2float(it)*M_PI/180.0f;
            }
        }
        ++it;
    }
    // camParam.PrintStuff();
    // SLAMParam.PrintStuff();
}

float Params::value2float(Json::ValueIterator & it) {
    return std::stof((*it)["value"].asString());
}