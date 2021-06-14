#include <spot/spot_check.h>

const static std::string CLIENT_NAME = "spot-check";

SpotCheckClient::SpotCheckClient(const std::string &authority, const std::string &token) : BaseClient(CLIENT_NAME, authority, token) {}