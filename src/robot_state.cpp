#include <spot/robot_state.h>


RobotStateClient::RobotStateClient(const std::string &root, const std::string &server) {
  
	// create options
  	grpc::SslCredentialsOptions opts;
  	opts.pem_root_certs = root;

	// create channel arguments
  	grpc::ChannelArguments channelArgs;
  	channelArgs.SetSslTargetNameOverride("robotstate.spot.robot"); // put into kv map later
  	stub_ = RobotStateService::NewStub(grpc::CreateCustomChannel(server, grpc::SslCredentials(opts), channelArgs));
} 

RobotStateClient::RobotStateClient(std::string token, const std::string &root, const std::string &server) {
  _stub = initialize(server, root, token, "state.spot.robot");
} 

// new 

RobotStateResponse RobotStateClient::getRobotState(std::string token){
  // Data we are sending to the server.
  RobotStateRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");

  return call<RobotStateRequest, RobotStateResponse>(request, &RobotStateService::Stub::GetRobotState);
}

RobotStateResponse RobotStateClient::getRobotStateAsync(){
  // Data we are sending to the server.
  RobotStateRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");

  return callAsync<RobotStateRequest, RobotStateResponse>(request, &RobotStateService::Stub::AsyncGetRobotState);

  // // Container for the data we expect from the server.
  // RobotStateResponse reply;

  // // Context for the client. It could be used to convey extra information to
  // // the server and/or tweak certain RPC behaviors.
  // ClientContext context;

  // // The actual RPC.
  // CompletionQueue cq;
  // std::unique_ptr<ClientAsyncResponseReader<RobotStateResponse>> rpc(stub_->AsyncGetRobotState(&context, request, &cq));

  // // status
  // Status status;
  // rpc->Finish(&reply, &status, (void*)1);

  // // Act upon its status.
  // if (status.ok()) {
  //   // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
  //   std::cout << "Success" << std::endl;
  // //   std::cout << reply.message() << std::endl;
  //   // return "reply.token()";
  // } else {
  //   std::cout << status.error_code() << ": " << status.error_message()
  //             << std::endl;
  //   // return "RPC failed";
  // }

  // return reply;
}

RobotMetricsResponse RobotStateClient::getRobotMetrics(){
  // Data we are sending to the server.
  RobotMetricsRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");

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

RobotMetricsResponse RobotStateClient::getRobotMetricsAsync(){
  // Data we are sending to the server.
  RobotMetricsRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");

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

RobotHardwareConfigurationResponse RobotStateClient::getRobotHardwareConfiguration(){
  // Data we are sending to the server.
  RobotHardwareConfigurationRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");

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

RobotHardwareConfigurationResponse RobotStateClient::getRobotHardwareConfigurationAsync(){
  // Data we are sending to the server.
  RobotHardwareConfigurationRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");

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

RobotLinkModelResponse RobotStateClient::getRobotLinkModel(const std::string &linkName){
  // Data we are sending to the server.
  RobotLinkModelRequest request;
  request.set_link_name(linkName);
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");

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

RobotLinkModelResponse RobotStateClient::getRobotLinkModelAsync(const std::string &linkName){
  // Data we are sending to the server.
  RobotLinkModelRequest request;
  request.set_link_name(linkName);
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");

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

RobotHardwareConfigurationResponse RobotStateClient::getHardwareConfigWithLinkInfo(){
  // Data we are sending to the server.
  RobotHardwareConfigurationRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");

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
