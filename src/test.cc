#include "../include/kitti_parser.h"
#include <iostream>
#include <vector>

using std::cout;
using std::vector;

int main(int argc, char* argv[]) {
    VeloParser parser("/home/yale/Documents/Slam/KittiData/VeloData/dataset/sequences/00/velodyne/000000.bin");
    if (! parser.ParseData())
       return 0;
    auto data = parser.GetData();
    for(auto i = 0; i < data.size(); ++ i) {
        cout << data[i].x << ' ' << data[i].y << ' ' << data[i].z << ' ' << data[i].reflectance << '\n';
    }
    return 0;
}

