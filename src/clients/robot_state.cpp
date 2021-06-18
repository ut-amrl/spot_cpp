#include <spot/clients/robot_state.h>

const std::string ROBOT_STATE_CLIENT_NAME = "robot-state";

RobotStateClient::RobotStateClient(const std::string &authority, const std::string &token) : BaseClient(ROBOT_STATE_CLIENT_NAME, authority, token) {}

RobotStateResponse RobotStateClient::getRobotState(){
  // Data we are sending to the server.
  RobotStateRequest request;
  assembleRequestHeader<RobotStateRequest>(&request);

  return call<RobotStateRequest, RobotStateResponse>(request, &RobotStateService::Stub::GetRobotState);
}

RobotStateResponse RobotStateClient::getRobotStateAsync(){
  // Data we are sending to the server.
  RobotStateRequest request;
  assembleRequestHeader<RobotStateRequest>(&request);

  return callAsync<RobotStateRequest, RobotStateResponse>(request, &RobotStateService::Stub::AsyncGetRobotState);
}

RobotMetricsResponse RobotStateClient::getRobotMetrics(){
  // Data we are sending to the server.
  RobotMetricsRequest request;
  assembleRequestHeader<RobotMetricsRequest>(&request);

  return call<RobotMetricsRequest, RobotMetricsResponse>(request, &RobotStateService::Stub::GetRobotMetrics);
}

RobotMetricsResponse RobotStateClient::getRobotMetricsAsync(){
  // Data we are sending to the server.
  RobotMetricsRequest request;
  assembleRequestHeader<RobotMetricsRequest>(&request);

  return callAsync<RobotMetricsRequest, RobotMetricsResponse>(request, &RobotStateService::Stub::AsyncGetRobotMetrics);
}

RobotHardwareConfigurationResponse RobotStateClient::getRobotHardwareConfiguration(){
  // Data we are sending to the server.
  RobotHardwareConfigurationRequest request;
  assembleRequestHeader<RobotHardwareConfigurationRequest>(&request);

  return call<RobotHardwareConfigurationRequest, RobotHardwareConfigurationResponse>(request, &RobotStateService::Stub::GetRobotHardwareConfiguration);
}

RobotHardwareConfigurationResponse RobotStateClient::getRobotHardwareConfigurationAsync(){
  // Data we are sending to the server.
  RobotHardwareConfigurationRequest request;
  assembleRequestHeader<RobotHardwareConfigurationRequest>(&request);

  return callAsync<RobotHardwareConfigurationRequest, RobotHardwareConfigurationResponse>(request, &RobotStateService::Stub::AsyncGetRobotHardwareConfiguration);
}

RobotLinkModelResponse RobotStateClient::getRobotLinkModel(const std::string &linkName){
  // Data we are sending to the server.
  RobotLinkModelRequest request;
  assembleRequestHeader<RobotLinkModelRequest>(&request);
  request.set_link_name(linkName);
  

  return call<RobotLinkModelRequest, RobotLinkModelResponse>(request, &RobotStateService::Stub::GetRobotLinkModel);
}

RobotLinkModelResponse RobotStateClient::getRobotLinkModelAsync(const std::string &linkName){
  // Data we are sending to the server.
  RobotLinkModelRequest request;
  assembleRequestHeader<RobotLinkModelRequest>(&request);
  request.set_link_name(linkName);

  return callAsync<RobotLinkModelRequest, RobotLinkModelResponse>(request, &RobotStateService::Stub::AsyncGetRobotLinkModel);
}