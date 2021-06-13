#ifndef AUTH_CLIENT_H
#define AUTH_CLIENT_H

#include <spot/base_client.h>
#include "bosdyn/api/auth_service.grpc.pb.h"


using grpc::Channel;
using grpc::ClientAsyncResponseReader;
using grpc::ClientContext;
using grpc::CompletionQueue;
using grpc::Status;

using bosdyn::api::GetAuthTokenRequest;
using bosdyn::api::GetAuthTokenResponse;
using bosdyn::api::AuthService;
using google::protobuf::util::TimeUtil;

class AuthClient : public BaseClient<AuthService> {
public:
  AuthClient(const std::string &root, const std::string &server);

  GetAuthTokenResponse auth(const std::string &user, const std::string &pass);
  GetAuthTokenResponse authAsync(const std::string &user, const std::string &pass);
  GetAuthTokenResponse authWithToken(const std::string &token);
  GetAuthTokenResponse authWithTokenAsync(const std::string &token);
};

#endif
