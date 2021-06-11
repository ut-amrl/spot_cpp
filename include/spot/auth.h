#ifndef AUTH_H
#define AUTH_H

#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <grpc++/grpc++.h>

#include "bosdyn/api/auth_service.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"
#include <google/protobuf/util/time_util.h>

using grpc::Channel;
using grpc::ClientAsyncResponseReader;
using grpc::ClientContext;
using grpc::CompletionQueue;
using grpc::Status;

using bosdyn::api::GetAuthTokenRequest;
using bosdyn::api::GetAuthTokenResponse;
using bosdyn::api::AuthService;
using google::protobuf::util::TimeUtil;

class AuthClient {
public:
  AuthClient(const std::string &root, const std::string &server);

  GetAuthTokenResponse auth(const std::string &user, const std::string &pass);
  GetAuthTokenResponse authAsync(const std::string &user, const std::string &pass);
  GetAuthTokenResponse authWithToken(const std::string &token);
  GetAuthTokenResponse authWithTokenAsync(const std::string &token);

 private:
  std::unique_ptr<AuthService::Stub> stub_;
};

#endif
