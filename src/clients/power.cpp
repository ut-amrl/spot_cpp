#include <spot/clients/power.h>

namespace ClientLayer {

  PowerClient::PowerClient(const std::string &authority, const std::string &token) : BaseClient(POWER_CLIENT_NAME, authority, token) {}

  PowerCommandResponse PowerClient::PowerCommand(bosdyn::api::Lease lease, const PowerCommandRequest_Request& powerRequest) {
    PowerCommandRequest request;
    assembleRequestHeader<PowerCommandRequest>(&request);
    request.mutable_lease()->CopyFrom(lease);
    request.set_request(powerRequest);
    return call<PowerCommandRequest, PowerCommandResponse>(request, &PowerService::Stub::PowerCommand);
  }


  PowerCommandResponse PowerClient::PowerCommandAsync(bosdyn::api::Lease lease, const PowerCommandRequest_Request& powerRequest) {
    PowerCommandRequest request;
    assembleRequestHeader<PowerCommandRequest>(&request);
    request.mutable_lease()->CopyFrom(lease);
    request.set_request(powerRequest);
    return callAsync<PowerCommandRequest, PowerCommandResponse>(request, &PowerService::Stub::AsyncPowerCommand);
  }



  PowerCommandFeedbackResponse PowerClient::PowerCommandFeedback(uint32_t powerCommandId) {
    PowerCommandFeedbackRequest request;
    assembleRequestHeader<PowerCommandFeedbackRequest>(&request);
    request.set_power_command_id(powerCommandId);
    return call<PowerCommandFeedbackRequest, PowerCommandFeedbackResponse>(request, &PowerService::Stub::PowerCommandFeedback);
  }


  PowerCommandFeedbackResponse PowerClient::PowerCommandFeedbackAsync(uint32_t powerCommandId) {
    PowerCommandFeedbackRequest request;
    assembleRequestHeader<PowerCommandFeedbackRequest>(&request);
    request.set_power_command_id(powerCommandId);
    return callAsync<PowerCommandFeedbackRequest, PowerCommandFeedbackResponse>(request, &PowerService::Stub::AsyncPowerCommandFeedback);
  }

};
