#include <spot/robot_id.h>

RobotIdClient::RobotIdClient(const std::string &server) {
	stub_ = RobotIdService::NewStub(grpc::CreateChannel(server, grpc::SslCredentials(grpc::SslCredentialsOptions())));
}

RobotIdClient::RobotIdClient(const std::string& root, const std::string& server) {
  grpc::SslCredentialsOptions opts;
  opts.pem_root_certs = root;
  //opts.pem_private_key = "";
  //opts.pem_cert_chain = "";
  grpc::ChannelArguments channelArgs;
  channelArgs.SetSslTargetNameOverride("id.spot.robot");
  stub_ = RobotIdService::NewStub(grpc::CreateCustomChannel(server, grpc::SslCredentials(opts), channelArgs));
}

RobotIdResponse RobotIdClient::getId(){
  // Data we are sending to the server.
  RobotIdRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");
  
  // Container for the data we expect from the server.
  RobotIdResponse reply;
  printf("%x\n", reply.robot_id().species());

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->GetRobotId(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Success" << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message() << std::endl;
  }

  printf("%x\n", reply.robot_id().species());
  std::cout << "Species: " << reply.robot_id().species() << std::endl;
  return reply;
}


RobotIdResponse RobotIdClient::getIdAsync(){
  // Data we are sending to the server.
  RobotIdRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  
  // Container for the data we expect from the server.
  RobotIdResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<RobotIdResponse>> rpc(stub_->AsyncGetRobotId(&context, request, &cq));

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
