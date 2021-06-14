/*
  power.h: includes client and interface for communication with the power service 
*/

#ifndef POWER_H
#define POWER_H

#include "bosdyn/api/power_service.grpc.pb.h"
#include "bosdyn/api/robot_state_service.grpc.pb.h"
#include <spot/base.h>

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

const extern std::string POWER_CLIENT_NAME;

class PowerClient : public BaseClient<PowerService> {
public:
  PowerClient(const std::string &authority, const std::string &token);

  PowerCommandResponse PowerCommand(Lease lease, const PowerCommandRequest_Request& request);
  PowerCommandResponse PowerCommandAsync(Lease lease, const PowerCommandRequest_Request& powerRequest);
  PowerCommandFeedbackResponse PowerCommandFeedback(uint32_t powerCommandId);
  PowerCommandFeedbackResponse PowerCommandFeedbackAsync(uint32_t powerCommandId);
};

#endif
