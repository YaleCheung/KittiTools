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
}

class ImuParser : KittiParser<ImuData> {
public:
    ImuParser() :
        KittiParser<ImuData>() {_ext = ".txt" }
    ImuParser(const fs::path& file_path) :
        KittiParser<ImuData>(file_path) { _ext = ".txt"; }

    auto ParseData(const fs::path& p) -> bool {
        SetPath(p);
        return _ParseData();
    }
private:
    auto _ParseData() -> bool {
        check_file_path();
        // return all the Imu data
    }
}

#endif // IMUPARSER_HHH
