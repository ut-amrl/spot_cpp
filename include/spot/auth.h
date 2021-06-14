/*
  auth.h: includes client and interface for communication with the authentication service 
*/

#ifndef AUTH_H
#define AUTH_H

#include <spot/base.h>
#include "bosdyn/api/auth_service.grpc.pb.h"

using bosdyn::api::GetAuthTokenRequest;
using bosdyn::api::GetAuthTokenResponse;
using bosdyn::api::AuthService;

class AuthClient : public BaseClient<AuthService> {
public:
  AuthClient();

  GetAuthTokenResponse auth(const std::string &user, const std::string &pass);
  GetAuthTokenResponse authAsync(const std::string &user, const std::string &pass);
  GetAuthTokenResponse authWithToken(const std::string &token);
  GetAuthTokenResponse authWithTokenAsync(const std::string &token);
};

#endif