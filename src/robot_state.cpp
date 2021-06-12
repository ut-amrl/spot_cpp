#include <spot/robot_state.h>


RobotStateClient::RobotStateClient(const std::string &root, const std::string &server) {
  _stub = initializeNoAuthToken(server, root, "state.spot.robot");
} 

RobotStateClient::RobotStateClient(std::string token, const std::string &root, const std::string &server) {
  _stub = initialize(server, root, token, "state.spot.robot");
} 

// new 

RobotStateResponse RobotStateClient::getRobotState(){
  // Data we are sending to the server.
  RobotStateRequest request;
  assembleRequestHeader<RobotStateRequest>(&request, "robot_state_client");

  return call<RobotStateRequest, RobotStateResponse>(request, &RobotStateService::Stub::GetRobotState);
}

RobotStateResponse RobotStateClient::getRobotStateAsync(){
  // Data we are sending to the server.
  RobotStateRequest request;
  assembleRequestHeader<RobotStateRequest>(&request, "robot_state_client");

  return callAsync<RobotStateRequest, RobotStateResponse>(request, &RobotStateService::Stub::AsyncGetRobotState);
}

RobotMetricsResponse RobotStateClient::getRobotMetrics(){
  // Data we are sending to the server.
  RobotMetricsRequest request;
  assembleRequestHeader<RobotMetricsRequest>(&request, "robot_state_client");

  return call<RobotMetricsRequest, RobotMetricsResponse>(request, &RobotStateService::Stub::GetRobotMetrics);
}

RobotMetricsResponse RobotStateClient::getRobotMetricsAsync(){
  // Data we are sending to the server.
  RobotMetricsRequest request;
  assembleRequestHeader<RobotMetricsRequest>(&request, "robot_state_client");

  return callAsync<RobotMetricsRequest, RobotMetricsResponse>(request, &RobotStateService::Stub::AsyncGetRobotMetrics);
}

RobotHardwareConfigurationResponse RobotStateClient::getRobotHardwareConfiguration(){
  // Data we are sending to the server.
  RobotHardwareConfigurationRequest request;
  assembleRequestHeader<RobotHardwareConfigurationRequest>(&request, "robot_state_client");

  return call<RobotHardwareConfigurationRequest, RobotHardwareConfigurationResponse>(request, &RobotStateService::Stub::GetRobotHardwareConfiguration);
}

RobotHardwareConfigurationResponse RobotStateClient::getRobotHardwareConfigurationAsync(){
  // Data we are sending to the server.
  RobotHardwareConfigurationRequest request;
  assembleRequestHeader<RobotHardwareConfigurationRequest>(&request, "robot_state_client");

  return callAsync<RobotHardwareConfigurationRequest, RobotHardwareConfigurationResponse>(request, &RobotStateService::Stub::AsyncGetRobotHardwareConfiguration);
}

RobotLinkModelResponse RobotStateClient::getRobotLinkModel(const std::string &linkName){
  // Data we are sending to the server.
  RobotLinkModelRequest request;
  assembleRequestHeader<RobotLinkModelRequest>(&request, "robot_state_client");
  request.set_link_name(linkName);
  

  return call<RobotLinkModelRequest, RobotLinkModelResponse>(request, &RobotStateService::Stub::GetRobotLinkModel);
}

RobotLinkModelResponse RobotStateClient::getRobotLinkModelAsync(const std::string &linkName){
  // Data we are sending to the server.
  RobotLinkModelRequest request;
  assembleRequestHeader<RobotLinkModelRequest>(&request, "robot_state_client");
  request.set_link_name(linkName);

  return callAsync<RobotLinkModelRequest, RobotLinkModelResponse>(request, &RobotStateService::Stub::AsyncGetRobotLinkModel);
}