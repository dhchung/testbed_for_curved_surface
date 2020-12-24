#include "data_load_module.h"

DataLoadModule::DataLoadModule(){
    path_initial_pose = "initial_pose.txt";
    path_absolute_measurement = "absolute_measurement.txt";
    path_plane_measurement = "plane_measurement.txt";
    path_odometry_measurement = "odometry_measurement.txt";
    path_gt_pose = "gt_pose.txt";
    path_gt_measurement = "gt_measurement.txt";

    initial_pose.clear();
    absolute_measurement.clear();
    plane_measurement.clear();
    odometry_measurement.clear();
    gt_pose.clear();
    gt_measurement.clear();

}
DataLoadModule::~DataLoadModule(){}

void DataLoadModule::LoadData(std::string data_path){
    initial_pose.clear();
    absolute_measurement.clear();
    plane_measurement.clear();
    odometry_measurement.clear();
    gt_pose.clear();
    gt_measurement.clear();

    std::string initial_pose_data_path = data_path+path_initial_pose;
    std::string absolute_measurement_data_path = data_path+path_absolute_measurement;
    std::string plane_measurement_data_path = data_path+path_plane_measurement;
    std::string odometry_measurement_data_path = data_path+path_odometry_measurement;
    std::string gt_pose_data_path = data_path+path_gt_pose;
    std::string gt_measurement_data_path = data_path+path_gt_measurement;

    insert_data(initial_pose, initial_pose_data_path);
    insert_data(absolute_measurement, absolute_measurement_data_path);
    insert_data(plane_measurement, plane_measurement_data_path);
    insert_data(odometry_measurement, odometry_measurement_data_path);
    insert_data(gt_pose, gt_pose_data_path);
    insert_data(gt_measurement, gt_measurement_data_path);

}

void DataLoadModule::split(const std::string &s, char delim, std::vector<float> &elems){
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while(std::getline(ss, item, delim)){
        elems.push_back(std::stof(item));
    }
}

void DataLoadModule::insert_data(std::vector<std::vector<float>> &data, std::string & data_path){
    std::ifstream infile(data_path);
    std::string line;
    while(std::getline(infile, line)) {
        std::vector<float> row_values;
        split(line, '\t', row_values);
        data.push_back(row_values);
    }
}