#ifndef IMUPARSER_HHH
#define IMUPARSER_HHH

#include "base_parser.h"
#include "../file_opt.h"

struct ImuData : public KittiData {
    // latitude logitude altitude
    double lat;
    double lon;
    double alt;
    // rad angles
    double roll;
    double pitch;
    double yaw;
    // velocity 
    double vn;
    double ve;
    double vf;
    double vl;
    double vu;
    // acceleration
    double ax;
    double ay;
    double az;
    double af;
    double al;
    double au;
    // angular rate
    double wx;
    double wy;
    double wz;
    double wf;
    double wl;
    double wu;
    // accuracy
    double pos_accuracy;
    double vel_accuracy;

    // state
    unsigned navstat;
    unsigned numsats;
    // mode
    unsigned int posmode;
    unsigned int velmode;
    unsigned int orimode;
};

class ImuParser : public KittiParser<ImuData> {
public:
    ImuParser() :
        KittiParser<ImuData>() {_ext = ".txt"; }
    ImuParser(const fs::path& file_path) :
        KittiParser<ImuData>(file_path) { _ext = ".txt"; }

    auto ParseData(const fs::path& p) -> bool {
        SetPath(p);
        return _ParseData();
    }
private:
    auto _ParseData() -> bool {
        FileOpt::CheckValidDir(_path_name);
        // return all the Imu data
        auto files = FileOpt::SubFiles(_path_name);
        
        for(const auto& file : files) {
            if (std::ifstream fin{file.c_str(), std::ios::in}) {
                std::stringstream ss;
                ImuData imu;
                ss << fin.rdbuf();

                ss >> imu.lat
                   >> imu.lon
                   >> imu.alt 
                   >> imu.roll
                   >> imu.pitch                       
                   >> imu.yaw
                   >> imu.vn
                   >> imu.ve
                   >> imu.vf
                   >> imu.vl
                   >> imu.vu
                   >> imu.ax
                   >> imu.ay
                   >> imu.az
                   >> imu.af
                   >> imu.al
                   >> imu.au
                   >> imu.wx
                   >> imu.wy
                   >> imu.wz
                   >> imu.wf
                   >> imu.wl
                   >> imu.wu
                   >> imu.pos_accuracy
                   >> imu.vel_accuracy
                   >> imu.navstat
                   >> imu.numsats
                   >> imu.posmode
                   >> imu.velmode
                   >> imu.orimode;
                _data.emplace_back(imu);
                fin.close();
            } else return false;
        }
        return true;
    }
};  //ImuParser
#endif // IMUPARSER_HHH
