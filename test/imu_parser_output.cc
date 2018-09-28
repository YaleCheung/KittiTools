#include "../include/parser.h"

#include <iostream>
#include <vector>
#include <cassert>

using std::cout;
using std::vector;

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "Please use <program> <imu_dir_path>" << '\n' ;
        return 0;
    }
    ImuParser parser;

    if (! parser.ParseData(argv[1]))
       return 0;
    auto data = parser.GetData();
    for(auto& ele : data) {
        std::cout << ele.lat << ' ' 
                  << ele.velmode << ' '
                  << ele.orimode << '\n';
    }
    return 0;
}
