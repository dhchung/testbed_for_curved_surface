#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iterator>
#include <sstream>

class DataLoadModule{
private:
    void split(const std::string &s, char delim, std::vector<float> &elems);
    void insert_data(std::vector<std::vector<float>> &data, std::string & data_path);

public:
    DataLoadModule();
    ~DataLoadModule();

    void LoadData(std::string data_path);

    std::string path_initial_pose;
    std::string path_absolute_measurement;
    std::string path_plane_measurement;
    std::string path_odometry_measurement;
    std::string path_gt_pose;
    std::string path_gt_measurement;
    

    std::vector<std::vector<float>> initial_pose;
    std::vector<std::vector<float>> absolute_measurement;
    std::vector<std::vector<float>> plane_measurement;
    std::vector<std::vector<float>> odometry_measurement;
    std::vector<std::vector<float>> gt_pose;
    std::vector<std::vector<float>> gt_measurement;

    
};