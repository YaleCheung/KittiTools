#ifndef FileOPT_HHH
#define FileOPT_HHH

#include <string>
#include <experimental/filesystem>
#include <vector>
#include <algorithm>
#include <memory>

namespace fs = std::experimental::filesystem;

class FileOPT {
public:
    FileOPT(const fs::path dir_name) : 
        _dir(dir_name), _file(new std::vector<fs::path>){
        _check_valid_dir(_dir);
        for(const auto path: fs::directory_iterator(_dir)) 
            _files -> push_back(path);

        std::sort(_files -> begin(), _files -> end());
    } 

    auto GetFiles() {
        return _files;
    }
private:
    bool _check_valid_dir() {
        fs::is_directory(_dir);
    }
    bool _check_valid_file(fs::path file) {
        return fs::is_regular_file(file);
    }
    fs::path _dir{""};
    std::shared_ptr<std::vector<fs::path>> _files;

};



#endif //FileOPT_HHH
