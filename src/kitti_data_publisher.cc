#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "../include/converter.h"

// only for clang c++17/14
//
#include <experimental/filesystem>

auto SubFiles(const fs::path& path) -> std::vector<fs::path> {
    std::vector<fs::path> subfiles;

    for(const auto& p : fs::directory_iterator(path)) 
        subfiles.push_back(p);
    std::sort(subfiles.begin(), subfiles.end());
    return subfiles;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_pcl");


    if (argc != 3) {
        ROS_ERROR("Usage:\n \ ¦       Odometry2Kitti <velo_data_pos> <time.txt pos>");
        return -1;
    }

    
    VeloConverter converter;

    VeloParser velo_parser;
    StampParser stamp_parser;

    stamp_parser.ParseData(argv[2]);

    // stamp_parser.ParseData();
    auto stamp_data = stamp_parser.GetData();
    //
    auto order_file = 0;
    auto  subfiles = SubFiles(argv[1]);

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise <sensor_msgs::PointCloud2> ("/kitti/velo/pointcloud", 5);
    // publish rate
    ros::Rate loop_rate(10);

    for(const auto& p : subfiles) {
        if (order_file % 100 == 0)
            ROS_INFO("Processing %dth cloud point patch", order_file);
      
            // velo_parser.SetPath(p);
        velo_parser.ParseData(p);
        auto velo_data = velo_parser.GetData();

        sensor_msgs::PointCloud2 msg;
        converter.PackPclmsg(velo_data, stamp_data, order_file, msg);        
        if (nh.ok()) {
            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
        ++ order_file;
    }
    
    return 0;
}
