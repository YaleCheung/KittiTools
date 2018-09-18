#ifndef FileOPT_HHH
#define FileOPT_HHH

#include <string>
#include <cassert>
#include <experimental/filesystem>
#include <vector>
#include <algorithm>
#include <memory>

namespace fs = std::experimental::filesystem;

class FileOPT {
public:

    static auto GetExtension(const fs::path& p) {
        assert(CheckValidFile(p));
        return p.extension();
    }

    static auto SubFiles(const fs::path& father = "") {
        assert(CheckValidDir(father));
        
        std::vector<fs::path> subfiles;
        for(const auto& son : fs::directory_iterator(father)) 
            subfiles.emplace_back(son);
        std::sort(subfiles.begin(), subfiles.end());
        return subfiles;
    }

    static bool CheckValidDir(const fs::path& p) {
        return fs::is_directory(p);
    }

    static bool CheckValidFile(const fs::path& p) {
        return fs::is_regular_file(p);
    }
};

#endif //FileOPT_HHH
