#include <iostream>
#include <fstream>
#include "data_load_module.h"

DataLoadModule dlm;

int main(){
    std::string data_path = "./Matlab/";
    dlm.LoadData(data_path);


    
    return 0;
}