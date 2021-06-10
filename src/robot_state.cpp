#include <spot/robot_state.h>

RobotState::RobotState(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) {
  grpc::SslCredentialsOptions opts = {root, key, cert};
  stub_ = RobotStateService::NewStub(grpc::CreateChannel(server, grpc::SslCredentials(opts)));
} 

// Assembles the client's payload, sends it and presents the response back
// from the server.
RobotStateResponse RobotState::GetRobotState() {
  // Data we are sending to the server.
  RobotStateRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  
  
  // Container for the data we expect from the server.
  RobotStateResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->GetRobotState(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
    std::cout << "Success" << std::endl;
  //   std::cout << reply.message() << std::endl;
    // return "reply.token()";
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    // return "RPC failed";
  }

  return reply;
}

// new 

RobotStateResponse RobotState::getRobotState(){
  // Data we are sending to the server.
  RobotStateRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());


  // Container for the data we expect from the server.
  RobotStateResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->GetRobotState(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
    std::cout << "Success" << std::endl;
  //   std::cout << reply.message() << std::endl;
    // return "reply.token()";
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    // return "RPC failed";
  }

  return reply;
}
RobotStateResponse RobotState::getRobotStateAsync(){
  // Data we are sending to the server.
  RobotStateRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());


  // Container for the data we expect from the server.
  RobotStateResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<RobotStateResponse>> rpc(stub_->AsyncGetRobotState(&context, request, &cq));

  // status
  Status status;
  rpc->Finish(&reply, &status, (void*)1);

  // Act upon its status.
  if (status.ok()) {
    // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
    std::cout << "Success" << std::endl;
  //   std::cout << reply.message() << std::endl;
    // return "reply.token()";
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    // return "RPC failed";
  }

  return reply;
}
RobotMetricsResponse RobotState::getRobotMetrics(){
  // Data we are sending to the server.
  RobotMetricsRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());


  // Container for the data we expect from the server.
  RobotMetricsResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->GetRobotMetrics(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
    std::cout << "Success" << std::endl;
  //   std::cout << reply.message() << std::endl;
    // return "reply.token()";
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    // return "RPC failed";
  }

  return reply;
}

RobotMetricsResponse RobotState::getRobotMetricsAsync(){
  // Data we are sending to the server.
  RobotMetricsRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());


  // Container for the data we expect from the server.
  RobotMetricsResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<RobotMetricsResponse>> rpc(stub_->AsyncGetRobotMetrics(&context, request, &cq));

  // status
  Status status;
  rpc->Finish(&reply, &status, (void*)1);

  // Act upon its status.
  if (status.ok()) {
    // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
    std::cout << "Success" << std::endl;
  //   std::cout << reply.message() << std::endl;
    // return "reply.token()";
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    // return "RPC failed";
  }

  return reply;

}

RobotHardwareConfigurationResponse RobotState::getRobotHardwareConfiguration(){
  // Data we are sending to the server.
  RobotHardwareConfigurationRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());


  // Container for the data we expect from the server.
  RobotHardwareConfigurationResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->GetRobotHardwareConfiguration(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
    std::cout << "Success" << std::endl;
  //   std::cout << reply.message() << std::endl;
    // return "reply.token()";
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    // return "RPC failed";
  }

  return reply;
}

RobotHardwareConfigurationResponse RobotState::getRobotHardwareConfigurationAsync(){
  // Data we are sending to the server.
  RobotHardwareConfigurationRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());


  // Container for the data we expect from the server.
  RobotHardwareConfigurationResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<RobotHardwareConfigurationResponse>> rpc(stub_->AsyncGetRobotHardwareConfiguration(&context, request, &cq));

  // status
  Status status;
  rpc->Finish(&reply, &status, (void*)1);

  // Act upon its status.
  if (status.ok()) {
    // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
    std::cout << "Success" << std::endl;
  //   std::cout << reply.message() << std::endl;
    // return "reply.token()";
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    // return "RPC failed";
  }

  return reply;
}

RobotLinkModelResponse RobotState::getRobotLinkModel(const std::string &linkName){
  // Data we are sending to the server.
  RobotLinkModelRequest request;
  request.set_link_name(linkName);
//  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());


  // Container for the data we expect from the server.
  RobotLinkModelResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->GetRobotLinkModel(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
    std::cout << "Success" << std::endl;
  //   std::cout << reply.message() << std::endl;
    // return "reply.token()";
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    // return "RPC failed";
  }

  return reply;
}

RobotLinkModelResponse RobotState::getRobotLinkModelAsync(const std::string &linkName){
  // Data we are sending to the server.
  RobotLinkModelRequest request;
  request.set_link_name(linkName);
//  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());


  // Container for the data we expect from the server.
  RobotLinkModelResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<RobotLinkModelResponse>> rpc(stub_->AsyncGetRobotLinkModel(&context, request, &cq));

  // status
  Status status;
  rpc->Finish(&reply, &status, (void*)1);

  // Act upon its status.
  if (status.ok()) {
    // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
    std::cout << "Success" << std::endl;
  //   std::cout << reply.message() << std::endl;
    // return "reply.token()";
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    // return "RPC failed";
  }

  return reply;
}

RobotHardwareConfigurationResponse RobotState::getHardwareConfigWithLinkInfo(){
  // Data we are sending to the server.
  RobotHardwareConfigurationRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());


  // Container for the data we expect from the server.
  RobotHardwareConfigurationResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->GetRobotHardwareConfiguration(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
    std::cout << "Success" << std::endl;
  //   std::cout << reply.message() << std::endl;
    // return "reply.token()";
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    // return "RPC failed";
  }

  return reply;
}
