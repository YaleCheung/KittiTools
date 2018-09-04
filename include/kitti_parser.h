#ifndef KittiParser_HHH
#define KittiParser_HHH

#include <fstream>
#include <iostream>
#include <experimental/filesystem>
#include <string>
#include <vector>
#include <cassert>
#include <memory>
#include <sstream>

namespace fs = std::experimental::filesystem;

using string = std::string;
using ifstream = std::ifstream;
using std::vector;
using std::cout;


// todo:  @@
//        @seperate kitti data from file operation;
//        @add more operations;
//        @@
// Problem:   not expception safe;    

// type ensurence
typedef struct KittyData {} KittiData;

typedef struct VelodyneData : public KittiData {
    float x;
    float y;
    float z;
    float reflectance;
} VelodyneData;

class Parser {
public:
    virtual bool ParseData() = 0; 
};

template<typename T, 
        class = typename std::enable_if<std::is_base_of<KittyData, T>::value>::type
        >
class KittiParser : Parser {
public:
    // constructor;
    KittiParser() :
        _path_name("") {};
    KittiParser(const fs::path file_path) :
        _path_name(file_path) {
        _data.reserve(10000);
    } 

    /* KittiParser(const fs::path file_path) : */
    /*     _path_name(file_path) { */
    /*     _data.reserve(10000); */
    /* } */
     
    auto GetPath() const { return _path_name.c_str(); }
    auto SetPath(fs::path file_path) {_path_name = file_path;}
    auto& GetData() const { return _data; }

    void check_path(fs::path _ext) {
        std::error_code ec; 
        assert(fs::is_regular_file(_path_name) && fs::exists(_path_name));
        // check bin file
        assert(_path_name.extension() == _ext);
    }
protected:    
    fs::path _path_name;
    vector<T> _data;
};// KittiParser;

class VeloParser : public KittiParser<VelodyneData> {
public:
    VeloParser() : 
        KittiParser<VelodyneData>() {};
    VeloParser(const fs::path file_path):
        KittiParser<VelodyneData>(file_path) { };
    bool ParseData() {
        // check the path  
        check_path(_ext);
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
            cout << __FILE__ << " " << __LINE__ << ": cannot open the file " << _path_name.c_str() << '\n';
            return false;
        }
        auto data = reinterpret_cast<VelodyneData*>(binary_data);
        auto end = data_length + binary_data;
         
        while((char*)data < end) {
            _data.push_back(*data);
            data += 1;
            //i += sizeof(VelodyneData);
        }

        delete [] binary_data;
        return true;
    }
    bool ParseData(const fs::path p) {
        SetPath(p);
        return ParseData();
    }
private:
    fs::path _ext{".bin"};
};


typedef struct StampData : public KittiData {
public:
    StampData(float stamp) :
        time_stamp(stamp) {}

    float time_stamp;

} StampData;
class StampParser : public KittiParser<StampData> {
public:
    StampParser() : 
        KittiParser<StampData>() {};
    StampParser(const fs::path file_path) :
        KittiParser<StampData>(file_path) {};
    bool ParseData() {
        check_path(_ext);
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
    bool ParseData(const fs::path p) {
        SetPath(p);
        return ParseData();
    }
private:
    const fs::path _ext{".txt"};

};
#endif // VeloParser
