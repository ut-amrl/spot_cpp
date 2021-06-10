#include <spot/robot_command.h>

RobotCommandClient::RobotCommandClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) {
  grpc::SslCredentialsOptions opts = {root, key, cert};
  stub_ = RobotCommandService::NewStub(grpc::CreateChannel(server, grpc::SslCredentials(opts)));
}
  
// Assembles the client's payload, sends it and presents the response back
// from the server.
RobotCommandResponse RobotCommandClient::startRobotCommand(Lease lease, RobotCommand command) {
  // Data we are sending to the server.
  RobotCommandClientRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_lease()->CopyFrom(lease);
  request.mutable_command()->CopyFrom(command);
  request.set_clock_identifier("spot_time_sync");
  
  // Container for the data we expect from the server.
  RobotCommandClientResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->RobotCommand(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Success" << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message() << std::endl;
  }

  return reply;
}


// Assembles the client's payload, sends it and presents the response back
// from the server. Asynchronous
RobotCommandResponse RobotCommandClient::robotCommandAsync(Lease lease, RobotCommand command){
  // Data we are sending to the server.
  RobotCommandClientRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_lease()->CopyFrom(lease);
  request.mutable_command()->CopyFrom(command);
  request.set_clock_identifier("spot_time_sync");
  
  // Container for the data we expect from the server.
  RobotCommandClientResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<RobotCommandResponse>> rpc(stub_->AsyncRobotCommand(&context, request, &cq));

  Status status;
  rpc->Finish(&reply, &status, (void*)1);

  void* got_tag;
  bool ok = false;
  if(!cq.Next(&got_tag, &ok)){
    std::cout << "Error: completion queue is fully drained" << std::endl;
  }
  else{
    if (ok && got_tag == (void*)1) {
      // Act upon its status.
      if (status.ok()) {
        std::cout << "Success" << std::endl;
      } else {
        std::cout << status.error_code() << ": " << status.error_message() << std::endl;
      }
    }
    else{
      std::cout << "Error: next value in completion queue does not match" << std::endl;
    }
  }

  return reply;
}

// Get feedback from a previously issued command.
RobotCommandFeedbackResponse RobotCommandClient::robotCommandFeedback(uint32_t robotCommandId){
  // Data we are sending to the server.
  RobotCommandClientFeedbackRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.set_robot_command_id(robotCommandId);
  
  // Container for the data we expect from the server.
  RobotCommandClientFeedbackResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->RobotCommandFeedback(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
    std::cout << "Success" << std::endl;
    // std::cout << reply.message() << std::endl;
    // return "reply.token()";
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    // return "RPC failed";
  }

  return reply;
}

// Get feedback from a previously issued command. Asynchronous
RobotCommandFeedbackResponse RobotCommandClient::robotCommandFeedbackAsync(uint32_t robotCommandId){
  // Data we are sending to the server.
  RobotCommandClientFeedbackRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.set_robot_command_id(robotCommandId);
  
  // Container for the data we expect from the server.
  RobotCommandClientFeedbackResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<RobotCommandFeedbackResponse>> rpc(stub_->AsyncRobotCommandFeedback(&context, request, &cq));

  Status status;
  rpc->Finish(&reply, &status, (void*)1);

  void* got_tag;
  bool ok = false;
  if(!cq.Next(&got_tag, &ok)){
    std::cout << "Error: completion queue is fully drained" << std::endl;
  }
  else{
    if (ok && got_tag == (void*)1) {
      // Act upon its status.
      if (status.ok()) {
        std::cout << "Success" << std::endl;
      } else {
        std::cout << status.error_code() << ": " << status.error_message() << std::endl;
      }
    }
    else{
      std::cout << "Error: next value in completion queue does not match" << std::endl;
    }
  }

  return reply;
}


// Clear robot behavior fault.
ClearBehaviorFaultResponse RobotCommandClient::clearBehaviorFault(Lease lease, uint32_t behaviorFaultId){
  // Data we are sending to the server.
  ClearBehaviorFaultRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_lease()->CopyFrom(lease);
  request.set_behavior_fault_id(behaviorFaultId);
  
  // Container for the data we expect from the server.
  ClearBehaviorFaultResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->ClearBehaviorFault(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
    std::cout << "Success" << std::endl;
    // std::cout << reply.message() << std::endl;
    // return "reply.token()";
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    // return "RPC failed";
  }

  return reply;
}


// Clear robot behavior fault. Async
ClearBehaviorFaultResponse RobotCommandClient::clearBehaviorFaultAsync(Lease lease, uint32_t behaviorFaultId){
  // Data we are sending to the server.
  ClearBehaviorFaultRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_lease()->CopyFrom(lease);
  request.set_behavior_fault_id(behaviorFaultId);
  
  // Container for the data we expect from the server.
  ClearBehaviorFaultResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<ClearBehaviorFaultResponse>> rpc(stub_->AsyncClearBehaviorFault(&context, request, &cq));

  Status status;
  rpc->Finish(&reply, &status, (void*)1);

  void* got_tag;
  bool ok = false;
  if(!cq.Next(&got_tag, &ok)){
    std::cout << "Error: completion queue is fully drained" << std::endl;
  }
  else{
    if (ok && got_tag == (void*)1) {
      // Act upon its status.
      if (status.ok()) {
        std::cout << "Success" << std::endl;
      } else {
        std::cout << status.error_code() << ": " << status.error_message() << std::endl;
      }
    }
    else{
      std::cout << "Error: next value in completion queue does not match" << std::endl;
    }
  }

  return reply;
}
