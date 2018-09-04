#include "../include/converter.h"

int main(int argc, char* argv[]) {
    if (argc < 4) {
        return -1;
    }
    VeloConverter converter(argv[1], argv[2], argv[3]);
    converter.ToBag();
    return 0;
}
