#include <spot/utils.h>

std::string read_file(const std::string &filename) {
    std::ifstream file(filename);
    std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return str;
}