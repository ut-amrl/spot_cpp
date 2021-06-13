#ifndef ROBOT_COMMAND_CLIENT_H
#define ROBOT_COMMAND_CLIENT_H

#include "bosdyn/api/robot_command_service.grpc.pb.h"
#include "bosdyn/api/geometry.grpc.pb.h"
#include "bosdyn/api/lease_service.grpc.pb.h"
#include <spot/base_client.h>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using grpc::CompletionQueue;
using grpc::ClientAsyncResponseReader;
using bosdyn::api::RobotCommandService;
using bosdyn::api::RobotCommandRequest;
using bosdyn::api::RobotCommandResponse;
using bosdyn::api::RobotCommandResponse_Status;
using bosdyn::api::RobotCommandFeedbackRequest;
using bosdyn::api::RobotCommandFeedbackResponse;
using bosdyn::api::ClearBehaviorFaultRequest;
using bosdyn::api::ClearBehaviorFaultResponse;
using bosdyn::api::RobotCommand;
using bosdyn::api::RobotCommandFeedback;
using bosdyn::api::FullBodyCommand;
using bosdyn::api::FullBodyCommand_Feedback;
using bosdyn::api::StopCommand;
using bosdyn::api::StopCommand_Feedback;
using bosdyn::api::FreezeCommand;
using bosdyn::api::FreezeCommand_Feedback;
using bosdyn::api::SelfRightCommand;
using bosdyn::api::SelfRightCommand_Feedback;
using bosdyn::api::SafePowerOffCommand;
using bosdyn::api::SafePowerOffCommand_Feedback;
using bosdyn::api::MobilityCommand;
using bosdyn::api::MobilityCommand_Feedback;
using bosdyn::api::SE2TrajectoryCommand;
using bosdyn::api::SE2TrajectoryCommand_Feedback;
using bosdyn::api::SE2VelocityCommand;
using bosdyn::api::SE2VelocityCommand_Feedback;
using bosdyn::api::SitCommand;
using bosdyn::api::SitCommand_Feedback;
using bosdyn::api::StandCommand;
using bosdyn::api::StandCommand_Feedback;
using bosdyn::api::Lease;
using bosdyn::api::LeaseUseResult;
using google::protobuf::Duration;
using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil;

class RobotCommandClient : public BaseClient<RobotCommandService> {
public:
  RobotCommandClient(const std::string &root, const std::string &server);

  RobotCommandResponse robotCommand(Lease lease, RobotCommand command);
  RobotCommandResponse robotCommandAsync(Lease lease, RobotCommand command);
  RobotCommandFeedbackResponse robotCommandFeedback(uint32_t robotCommandId);
  RobotCommandFeedbackResponse robotCommandFeedbackAsync(uint32_t robotCommandId);
  ClearBehaviorFaultResponse clearBehaviorFault(Lease lease, uint32_t behaviorFaultId);
  ClearBehaviorFaultResponse clearBehaviorFaultAsync(Lease lease, uint32_t behaviorFaultId);

private:
  //new
  RobotCommandRequest getRobotCommandRequest(Lease lease, RobotCommand command);
  RobotCommandFeedbackRequest getRobotCommandFeedbackRequest(uint32_t robotCommandId);
  ClearBehaviorFaultRequest getClearBehaviorFaultRequest(Lease lease, uint32_t behaviorFaultId);
};

#endif
