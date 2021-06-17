#include <spot/utils.h>

std::string read_file(const std::string &filename) {
    std::ifstream file(filename);
    std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return str;
}

std::string DEFAULT_SPOT_SERVER = "192.168.80.3";
std::string DEFAULT_SECURE_PORT = ":443";

std::string DEFAULT_ROOT_CERT_FILEPATH = "../src/resources/robot.pem";
std::string DEFAULT_ROOT_CERT = read_file(DEFAULT_ROOT_CERT_FILEPATH);