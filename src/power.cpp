#include <spot/power.h>

PowerClient::PowerClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) {
  grpc::SslCredentialsOptions opts = {root, key, cert};
  stub_ = PowerService::NewStub(grpc::CreateChannel(server, grpc::SslCredentials(opts)));
}

PowerCommandResponse PowerClient::PowerCommand(Lease lease, PowerCommandRequest_Request powerRequest) {
  // Data we are sending to the server.
  PowerCommandRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");
  request.mutable_lease()->CopyFrom(lease);
  request.set_request(powerRequest);
  
  // Container for the data we expect from the server.
  PowerCommandResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->PowerCommand(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Power Command Status: " << reply.status() << std::endl;
    return reply;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    return reply;
  }
}


PowerCommandResponse PowerClient::PowerCommandAsync(Lease lease, PowerCommandRequest_Request powerRequest) {
  // Data we are sending to the server.
  PowerCommandRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");
  request.mutable_lease()->CopyFrom(lease);
  request.set_request(powerRequest);

  // Container for the data we expect from the server.
  PowerCommandResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<PowerCommandResponse>> rpc(stub_->AsyncPowerCommand(&context, request, &cq));

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



PowerCommandFeedbackResponse PowerClient::PowerCommandFeedback(uint32_t powerCommandId) {
  // Data we are sending to the server.
  PowerCommandFeedbackRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");
  request.set_power_command_id(powerCommandId);
  
  // Container for the data we expect from the server.
  PowerCommandFeedbackResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->PowerCommandFeedback(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Power Command Feedback Status: " << reply.status() << std::endl;
    return reply;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    return reply;
  }
}


PowerCommandFeedbackResponse PowerClient::PowerCommandFeedbackAsync(uint32_t powerCommandId) {
  // Data we are sending to the server.
  PowerCommandFeedbackRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");
  request.set_power_command_id(powerCommandId);
  
  // Container for the data we expect from the server.
  PowerCommandFeedbackResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<PowerCommandFeedbackResponse>> rpc(stub_->AsyncPowerCommandFeedback(&context, request, &cq));

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
