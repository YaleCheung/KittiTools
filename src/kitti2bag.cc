#include "../include/converter.h"

int main(int argc, char* argv[]) {
    if (argc < 4) {
        ROS_ERROR("Usage:\n \
                Odometry2Kitti <bagfile_pos> <velo_data_pos> <time.txt pos>");
        return -1;
    }
    VeloConverter converter(argv[1], argv[2], argv[3]);
    converter.ToBag();
    return 0;
}
