/*
  auth.h: includes client for communication with the authentication service 
*/

#ifndef AUTH_H
#define AUTH_H

#include <spot/clients/base.h>
#include "bosdyn/api/auth_service.grpc.pb.h"

using bosdyn::api::GetAuthTokenRequest;
using bosdyn::api::GetAuthTokenResponse;
using bosdyn::api::AuthService;

namespace ClientLayer {

  class InvalidCredentialsError : ResponseError<GetAuthTokenResponse> {
  public:
    InvalidCredentialsError(GetAuthTokenResponse &response, const std::string &message) : ResponseError<GetAuthTokenResponse>(response, message) {}
  };

  class InvalidTokenError : ResponseError<GetAuthTokenResponse> {
  public:
    InvalidTokenError(GetAuthTokenResponse &response, const std::string &message) : ResponseError<GetAuthTokenResponse>(response, message) {}
  };

  class AuthClient : public BaseClient<AuthService> {
  public:
    AuthClient();

    // rpcs
    GetAuthTokenResponse auth(const std::string &user, const std::string &pass);
    GetAuthTokenResponse authAsync(const std::string &user, const std::string &pass);
    GetAuthTokenResponse authWithToken(const std::string &token);
    GetAuthTokenResponse authWithTokenAsync(const std::string &token);
  };

};
#endif
