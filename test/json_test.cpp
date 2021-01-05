#include <iostream>
#include "parameters_json.h"

int main()
{
    Params params;

    std::string data_dir = "./include/parameters.json";
    params.read_data(data_dir);

    return 0;
}