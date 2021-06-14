#include <spot/robot_command.h>

const std::string ROBOT_COMMAND_CLIENT_NAME = "robot-command";

RobotCommandClient::RobotCommandClient(const std::string &authority, const std::string &token) : BaseClient(ROBOT_COMMAND_CLIENT_NAME, authority, token) {}
  
// Assembles the client's payload, sends it and presents the response back
// from the server.
RobotCommandResponse RobotCommandClient::robotCommand(Lease lease, RobotCommand command, std::string &clockIdentifier) {
  RobotCommandRequest request;
  assembleRequestHeader<RobotCommandRequest>(&request);
  request.mutable_lease()->CopyFrom(lease);
  request.mutable_command()->CopyFrom(command);
  request.set_clock_identifier(clockIdentifier);
  return call<RobotCommandRequest, RobotCommandResponse>(request, &RobotCommandService::Stub::RobotCommand);
}


// Assembles the client's payload, sends it and presents the response back
// from the server. Asynchronous
RobotCommandResponse RobotCommandClient::robotCommandAsync(Lease lease, RobotCommand command, std::string &clockIdentifier){
  RobotCommandRequest request;
  assembleRequestHeader<RobotCommandRequest>(&request);
  request.mutable_lease()->CopyFrom(lease);
  request.mutable_command()->CopyFrom(command);
  request.set_clock_identifier(clockIdentifier);
  return callAsync<RobotCommandRequest, RobotCommandResponse>(request, &RobotCommandService::Stub::AsyncRobotCommand);
}

// Get feedback from a previously issued command.
RobotCommandFeedbackResponse RobotCommandClient::robotCommandFeedback(uint32_t robotCommandId){
  RobotCommandFeedbackRequest request;
  request.set_robot_command_id(robotCommandId);
  assembleRequestHeader<RobotCommandFeedbackRequest>(&request);
  return call<RobotCommandFeedbackRequest, RobotCommandFeedbackResponse>(request, &RobotCommandService::Stub::RobotCommandFeedback);
}

// Get feedback from a previously issued command. Asynchronous
RobotCommandFeedbackResponse RobotCommandClient::robotCommandFeedbackAsync(uint32_t robotCommandId){
  RobotCommandFeedbackRequest request;
  request.set_robot_command_id(robotCommandId);
  assembleRequestHeader<RobotCommandFeedbackRequest>(&request);
  return callAsync<RobotCommandFeedbackRequest, RobotCommandFeedbackResponse>(request, &RobotCommandService::Stub::AsyncRobotCommandFeedback);
}


// Clear robot behavior fault.
ClearBehaviorFaultResponse RobotCommandClient::clearBehaviorFault(Lease lease, uint32_t behaviorFaultId){
  ClearBehaviorFaultRequest request;
  request.mutable_lease()->CopyFrom(lease);
  request.set_behavior_fault_id(behaviorFaultId);
  assembleRequestHeader<ClearBehaviorFaultRequest>(&request);
  return call<ClearBehaviorFaultRequest, ClearBehaviorFaultResponse>(request, &RobotCommandService::Stub::ClearBehaviorFault);
}


// Clear robot behavior fault. Async
ClearBehaviorFaultResponse RobotCommandClient::clearBehaviorFaultAsync(Lease lease, uint32_t behaviorFaultId){
  ClearBehaviorFaultRequest request;
  request.mutable_lease()->CopyFrom(lease);
  request.set_behavior_fault_id(behaviorFaultId);
  assembleRequestHeader<ClearBehaviorFaultRequest>(&request);
  return callAsync<ClearBehaviorFaultRequest, ClearBehaviorFaultResponse>(request, &RobotCommandService::Stub::AsyncClearBehaviorFault);
}
