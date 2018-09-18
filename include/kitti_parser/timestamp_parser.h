#ifndef TIMESTAMPPARSER_HHH
#define TIMESTAMPPARSER_HHH

#include "base_parser.h"

typedef struct StampData : public KittiData {
public:
    StampData(double stamp) :
        time_stamp(stamp) {}

    double time_stamp;

} StampData;

class StampParser : public KittiParser<StampData> {
public:
    StampParser() :
        KittiParser<StampData>() { _ext = ".txt"; }
    StampParser(const fs::path& file_path) :
        KittiParser<StampData>(file_path){_ext = ".txt";}
    auto ParseData(const fs::path& p) -> bool {
        SetPath(p);
        return _ParseData();
    }
private:
    auto _ParseData() -> bool{
        check_file_path();
        if(std::ifstream fin{_path_name.c_str(), std::ios::in}) {
            std::stringstream str_stream;
            str_stream << fin.rdbuf();
            float input;
            while(str_stream >> input)
               _data.emplace_back(StampData(input));
            fin.close();
            return true;
        }
        return false;
    }
};   // stampparser
#endif // TIMESTAMPPARSER_HHH
