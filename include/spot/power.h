#ifndef POWER_H
#define POWER_H

#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <grpc++/grpc++.h>

#include "bosdyn/api/power_service.grpc.pb.h"
#include "bosdyn/api/robot_state_service.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"
#include <google/protobuf/util/time_util.h>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using grpc::CompletionQueue;
using grpc::ClientAsyncResponseReader;

using bosdyn::api::PowerCommandRequest;
using bosdyn::api::PowerCommandRequest_Request;
using bosdyn::api::PowerCommandResponse;
using bosdyn::api::PowerCommandFeedbackRequest;
using bosdyn::api::PowerCommandFeedbackResponse;
using bosdyn::api::PowerService;
using bosdyn::api::PowerCommandStatus;
using bosdyn::api::PowerState;
using bosdyn::api::PowerState_MotorPowerState;
using bosdyn::api::Lease;
using bosdyn::api::LeaseUseResult;
using bosdyn::api::LicenseInfo;
using google::protobuf::util::TimeUtil;

class PowerClient {
public:
  PowerClient(const std::string &root, const std::string &server);

  PowerCommandResponse powerCommand(const PowerCommandRequest& request);
  PowerCommandResponse PowerCommandAsync(Lease lease, PowerCommandRequest_Request powerRequestt);
  PowerCommandFeedbackResponse powerCommandFeedback(uint32_t powerCommandId);
  PowerCommandFeedbackResponse PowerCommandFeedbackAsync(uint32_t powerCommandId);

 private:
  std::unique_ptr<PowerService::Stub> stub_;
};

#endif
