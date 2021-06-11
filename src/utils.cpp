#include <spot/utils.h>

std::map<std::string, std::string> BOOTSTRAP_SERVICE_AUTHORITIES = {
    {"robot_id", "id.spot.robot"},
};

std::string DEFAULT_SECURE_CHANNEL_PORT = "443";

void read_file(const std::string &filename, std::string &data) {
    std::ifstream file(filename.c_str(), std::ios::in);
    if (file.is_open()) {
        std::stringstream ss;
        ss << file.rdbuf();
        file.close();
        data = ss.str();
    }
}