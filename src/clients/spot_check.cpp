#include <spot/clients/spot_check.h>

namespace ClientLayer {

    SpotCheckClient::SpotCheckClient(const std::string &authority, const std::string &token) : BaseClient(SPOT_CHECK_CLIENT_NAME, authority, token) {}

};