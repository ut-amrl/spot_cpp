#include <spot/clients/spot_check.h>

const std::string SPOT_CHECK_CLIENT_NAME = "spot-check";

SpotCheckClient::SpotCheckClient(const std::string &authority, const std::string &token) : BaseClient(SPOT_CHECK_CLIENT_NAME, authority, token) {}