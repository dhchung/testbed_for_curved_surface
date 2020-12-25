#include <iostream>
#include <fstream>
#include "data_load_module.h"
#include "plane_measure_factor.h"
#include "surfel_node.h"

DataLoadModule dlm;

int main(){
    std::string data_path = "./Matlab/";
    dlm.LoadData(data_path);


    
    return 0;
}