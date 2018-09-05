#ifndef KITTIDATA_HHH
#define KITTIDATA_HHH

#include "kitti_parser.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <string>
#include <map>
#include <algorithm>
#include <ctime>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <experimental/filesystem>
#include "noncopyable.h"

namespace fs = std::experimental::filesystem;

class BaseConverter : public NonCopyable {
public:
    virtual auto ToBag() -> bool = 0;
};

class VeloConverter : BaseConverter {
public:
    VeloConverter(const fs::path& bag_file_name, const fs::path& data_dir, const fs::path& timestamp_path) :
       _bag_file_path(bag_file_name), 
       _velo_data_dir(data_dir), 
       _timestamp_path(timestamp_path) { }


    auto ToBag() -> bool {
        _bag.open(_bag_file_path.c_str(), rosbag::bagmode::Write);
        if (! fs::exists(_bag_file_path.c_str())) 
            return false;

        VeloParser velo_parser;
        StampParser stamp_parser;
        stamp_parser.ParseData(_timestamp_path);

        // stamp_parser.ParseData();
        auto stamp_data = stamp_parser.GetData();

        auto order_file = 0;
        auto subfiles = _SubFiles(_velo_data_dir);

        for(auto& p : subfiles) {
            if (order_file % 100 == 0)
                ROS_INFO("Processing %dth file\t  Total:%d", order_file, subfiles.size());
           
            // velo_parser.SetPath(p);
            velo_parser.ParseData(p); 
            auto velo_data = velo_parser.GetData();
            // envo the constract;
            std_msgs::Header  header;
            header.frame_id = _frame_id;
            double time = stamp_data[order_file ++].time_stamp;
            header.stamp = ros::Time().fromSec(1 + time);
           
            //fill the pointcloud2 msg
            sensor_msgs::PointCloud2 msg;
            msg.fields.resize(4);
            
            // field 0;
            msg.fields[0].name = "x";
            msg.fields[0].offset = 0;
            msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
            msg.fields[0].count = 1;
            
            // field 1;
            msg.fields[1].name = "y";
            msg.fields[1].offset = 4;
            msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
            msg.fields[1].count = 1;

            // field 2;
            msg.fields[2].name = "z";
            msg.fields[2].offset = 8;
            msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
            msg.fields[2].count = 1;
            // field 3;
            msg.fields[3].name = "i";
            msg.fields[3].offset = 12;
            msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
            msg.fields[3].count = 1;
            
            // package a msg
            msg.header = header;
            msg.height = 1;
            msg.width = velo_data.size();
            msg.is_dense = false;
            msg.is_bigendian = false;
            /* msg.fields = fields; */
            msg.point_step = POINTSTEP;
            msg.data.resize(std::max((size_t)1, velo_data.size()) * POINTSTEP, 0x00);
            uint8_t* ptr = msg.data.data();
            for (auto i = 0; i < velo_data.size(); ++ i) {
                 *((float*)(ptr + 0)) = velo_data[i].x;
                 *((float*)(ptr + 4)) = velo_data[i].y;
                 *((float*)(ptr + 8)) = velo_data[i].z;
                 *((float*)(ptr + 12)) = velo_data[i].reflectance;
                 ptr += POINTSTEP;
            }

            _bag.write(_topic, header.stamp, msg);
        }
        _bag.close();
        return true;
    }
    // return data from a seq
    
     


private:
    // will be optimized by clang for the RVO;
    std::vector<fs::path> _SubFiles(const fs::path& path) {
        vector<fs::path> subfiles;
        for(const auto& p : fs::directory_iterator(path)) {
            subfiles.push_back(p);
        }
        std::sort(subfiles.begin(), subfiles.end());
        return subfiles;
    }

    // auto _LoadVeloDataSet(const fs::path& p) {
    //     VeloParser parser;
    //     parser.SetPath(p);
    //     parser.ParseData();
    //     auto data = parser.GetData();
    //     return data;
    // }
    //
    // // where copy happens, should be optimized by rref
    // auto _LoadStampDataset(const fs::path& p) {
    //     StampParser parser;
    //     parser.SetPath(p);
    //     parser.ParseData();
    //     auto data = parser.GetData();
    //     return data;
    // }

    typename fs::path _velo_data_dir{""};
    typename fs::path _timestamp_path{""};

    std::vector<fs::path> _velo_files;
    std::vector<fs::path> _stamp_files;

    typename rosbag::Bag _bag;
    fs::path _bag_file_path;
    
    std::string _topic{"/kitti/velo/pointcloud"};
    std::string _frame_id{"velo_link"};
    const uint32_t POINTSTEP{16};
};

#endif //KITTIDATA_HHH
