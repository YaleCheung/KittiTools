#ifndef VELOPASER_HHH
#define VELOPASER_HHH

#include "base_parser.h"

typedef struct VelodyneData : public KittiData {
    float x;
    float y;
    float z;
    float reflectance;
} VelodyneData;

class VeloParser : public KittiParser<VelodyneData> {
public:
    VeloParser() :
        KittiParser<VelodyneData>() { _ext = ".bin"; }
    VeloParser(const fs::path& file_path):
        KittiParser<VelodyneData>(file_path){ _ext = ".bin"; }
    auto ParseData(const fs::path& p) -> bool {
        SetPath(p);
        return _ParseData();
    }
private:
    auto _ParseData() -> bool {
        // check the path
        check_file_path();
        // cloud be used to managed by shared_ptr since cpp17, for the auto destroy ability;
        char* binary_data = nullptr;   // binary data;
        // load data
        auto data_length = 0;
        if (std::ifstream fin{_path_name.c_str(), std::ios::binary | std::ios::ate}) {
            data_length = fin.tellg();
            binary_data = new char[data_length];
            fin.seekg(0);
            fin.read(binary_data, data_length);
            fin.close();
        } else {
            std::cout << __FILE__ << " " << __LINE__ << ": cannot open the file " << _path_name.c_str() << '\n';
            return false;
        }
        auto velo_data = reinterpret_cast<VelodyneData*>(binary_data);
        auto end = data_length + binary_data;

        while((char*)velo_data < end) {
            _data.push_back(* velo_data);
            velo_data += 1;
            //i += sizeof(VelodyneData);
        }

        delete [] binary_data;
        return true;
    }
};

#endif 
