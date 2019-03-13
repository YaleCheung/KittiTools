#ifndef KITTIPARSER_HHH
#define KITTIPARSER_HHH

#include <fstream>
#include <iostream>
#include <experimental/filesystem>
#include <string>
#include <vector>
#include <cassert>
#include <memory>
#include <sstream>

#include "../noncopyable.h"
namespace fs = std::experimental::filesystem;



// todo:  @@
//        @seperate kitti data from file operation;
//        @add more operations;
//        @@
// Problem:   non expception safe;    

// type ensurence
class Parser : public NonCopyable {
public:
    virtual auto ParseData(const fs::path&) -> bool = 0; 
};


typedef struct KittiData {} KittiData;


template<typename T, 
        class = typename std::enable_if<std::is_base_of<KittiData, T>::value>::type
        >
class KittiParser : public Parser {
public:
    // constructor;
    KittiParser() :
        _path_name(""), _ext("") { _data.reserve(6000); };
    KittiParser(const fs::path& file_path) :
        _path_name(file_path) , _ext("") {
        
        _data.reserve(10000);
    } 

    auto GetPath() const { return _path_name.c_str(); }
    auto SetPath(const fs::path& file_path) {
        _data.clear();
        _path_name = file_path;
    }
    auto& GetData() const { return _data; }

    auto check_file_path() const {
        std::error_code ec; 
        assert(fs::is_regular_file(_path_name) && fs::exists(_path_name));
        // check bin file
        assert(_path_name.extension() == _ext);
    }
protected:    
    fs::path _path_name;
    fs::path _ext;
    std::vector<T> _data;
};// KittiParser;

#endif // VELOPARSER_HHH
