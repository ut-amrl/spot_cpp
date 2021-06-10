#include <spot/estop.h>

EstopClient::EstopClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) {
  grpc::SslCredentialsOptions opts = {root, key, cert};
  stub_ = EstopService::NewStub(grpc::CreateChannel(server, grpc::SslCredentials(opts)));
}

// Assembles the client's payload, sends it and presents the response back
// from the server.
std::string EstopClient::RegisterEstopEndpoint(EstopEndpoint new_endpoint) {
  // Data we are sending to the server.
  RegisterEstopEndpointRequest request;
  request.mutable_new_endpoint()->CopyFrom(new_endpoint);
  std::cout << "Endpoint unique ID: " << request.new_endpoint().unique_id() << std::endl;

  // Container for the data we expect from the server.
  RegisterEstopEndpointResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->RegisterEstopEndpoint(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << reply.status() << ", Response: " << reply.new_endpoint().unique_id() << std::endl;
    std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
    return reply.new_endpoint().unique_id();
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    return "RPC failed";
  }
}
 
RegisterEstopEndpointResponse EstopClient::registerEndpoint(const std::string &targetConfigId, EstopEndpoint &endpoint) {
	RegisterEstopEndpointRequest request;
	request.mutable_new_endpoint()->CopyFrom(endpoint);

	RegisterEstopEndpointResponse reply;
	ClientContext context;
	Status status = stub_->RegisterEstopEndpoint(&context, request, &reply);

	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
	    	std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	return reply;
}

RegisterEstopEndpointResponse EstopClient::registerEndpointAsync(const std::string &targetConfigId, EstopEndpoint &endpoint) {
	RegisterEstopEndpointRequest request;
	request.mutable_new_endpoint()->CopyFrom(endpoint);
	
	RegisterEstopEndpointResponse reply;
	ClientContext context;
	CompletionQueue cq;
	Status status;
	std::unique_ptr<ClientAsyncResponseReader<RegisterEstopEndpointResponse>> rpc(
			stub_->PrepareAsyncRegisterEstopEndpoint(&context, request, &cq));
	rpc->StartCall();

	rpc->Finish(&reply, &status, (void *)1);
	void *got_tag;
	bool ok = false;

	GPR_ASSERT(cq.Next(&got_tag, &ok));
	GPR_ASSERT(got_tag == (void *)1);
	GPR_ASSERT(ok);

	// act upon rpc status
	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
	    	std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	// return response
	return reply;
}

DeregisterEstopEndpointResponse EstopClient::deregister(const std::string &targetConfigId, EstopEndpoint &endpoint) {
	// populate request
	DeregisterEstopEndpointRequest request;
	request.mutable_target_endpoint()->CopyFrom(endpoint);
	request.set_target_config_id(targetConfigId);

	DeregisterEstopEndpointResponse reply;
	ClientContext context;
	Status status = stub_->DeregisterEstopEndpoint(&context, request, &reply);

	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
		std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	return reply;
}

DeregisterEstopEndpointResponse EstopClient::deregisterAsync(const std::string &targetConfigId, EstopEndpoint &endpoint) {
	DeregisterEstopEndpointRequest request;
	request.mutable_target_endpoint()->CopyFrom(endpoint);
	request.set_target_config_id(targetConfigId);

	DeregisterEstopEndpointResponse reply;
	ClientContext context;
	CompletionQueue cq;
	Status status;

	std::unique_ptr<ClientAsyncResponseReader<DeregisterEstopEndpointResponse>> rpc(
			stub_->PrepareAsyncDeregisterEstopEndpoint(&context, request, &cq));
	rpc->StartCall();

	rpc->Finish(&reply, &status, (void *)1);
	void *got_tag;
	bool ok = false;
	GPR_ASSERT(cq.Next(&got_tag, &ok));
	GPR_ASSERT(got_tag == (void *)1);
	GPR_ASSERT(ok);

	// check status
	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
		std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	return reply;
}

GetEstopConfigResponse EstopClient::getConfig(const std::string &targetConfigId) {
	GetEstopConfigRequest request;
	request.set_target_config_id(targetConfigId);

	GetEstopConfigResponse reply;
	ClientContext context;
	Status status = stub_->GetEstopConfig(&context, request, &reply);

	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
		std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	return reply;
}

GetEstopConfigResponse EstopClient::getConfigAsync(const std::string &targetConfigId) {
	GetEstopConfigRequest request;
	request.set_target_config_id(targetConfigId);

	GetEstopConfigResponse reply;
	ClientContext context;
	CompletionQueue cq;
	Status status;

	std::unique_ptr<ClientAsyncResponseReader<GetEstopConfigResponse>> rpc(
			stub_->PrepareAsyncGetEstopConfig(&context, request, &cq));
	rpc->StartCall();

	rpc->Finish(&reply, &status, (void *)1);
	void *got_tag;
	bool ok = false;
	GPR_ASSERT(cq.Next(&got_tag, &ok));
	GPR_ASSERT(got_tag == (void *)1);
	GPR_ASSERT(ok);

	// check status
	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
		std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	return reply;
}

SetEstopConfigResponse EstopClient::setConfig(EstopConfig &config, std::string targetConfigId) {
	SetEstopConfigRequest request;
	request.mutable_config()->CopyFrom(config);
	request.set_target_config_id(targetConfigId);

	SetEstopConfigResponse reply;
	ClientContext context;
	Status status = stub_->SetEstopConfig(&context, request, &reply);

	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
		std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	return reply;
}

SetEstopConfigResponse EstopClient::setConfigAsync(EstopConfig &config, std::string targetConfigId) {
	SetEstopConfigRequest request;
	request.mutable_config()->CopyFrom(config);
	request.set_target_config_id(targetConfigId);

	SetEstopConfigResponse reply;
	ClientContext context;
	CompletionQueue cq;
	Status status;

	std::unique_ptr<ClientAsyncResponseReader<SetEstopConfigResponse>> rpc(
			stub_->PrepareAsyncSetEstopConfig(&context, request, &cq));
	rpc->StartCall();

	rpc->Finish(&reply, &status, (void *)1);
	void *got_tag;
	bool ok = false;
	GPR_ASSERT(cq.Next(&got_tag, &ok));
	GPR_ASSERT(got_tag == (void *)1);
	GPR_ASSERT(ok);

	// check status
	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
		std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	return reply;
}

GetEstopSystemStatusResponse EstopClient::getStatus() {
	GetEstopSystemStatusRequest request;

	GetEstopSystemStatusResponse reply;
	ClientContext context;
	Status status = stub_->GetEstopSystemStatus(&context, request , &reply);

	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
		std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	return reply;
}

GetEstopSystemStatusResponse EstopClient::getStatusAsync() {
	GetEstopSystemStatusRequest request;

	GetEstopSystemStatusResponse reply;
	ClientContext context;
	CompletionQueue cq;
	Status status;

	std::unique_ptr<ClientAsyncResponseReader<GetEstopSystemStatusResponse>> rpc(
			stub_->PrepareAsyncGetEstopSystemStatus(&context, request, &cq));
	rpc->StartCall();

	rpc->Finish(&reply, &status, (void *)1);
	void *got_tag;
	bool ok = false;
	GPR_ASSERT(cq.Next(&got_tag, &ok));
	GPR_ASSERT(got_tag == (void *)1);
	GPR_ASSERT(ok);

	// check status
	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
		std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	return reply;
}

EstopCheckInResponse EstopClient::checkIn(EstopStopLevel &stopLevel, EstopEndpoint &endpoint, uint64_t challenge, uint64_t response, bool suppress_incorrect) {
	// TODO: implement logic for suppress_incorrect
	EstopCheckInRequest request;

	request.set_challenge(challenge);
	request.set_response(response);
	request.mutable_endpoint()->CopyFrom(endpoint);
	request.set_stop_level(stopLevel);

	EstopCheckInResponse reply;
	ClientContext context;
	Status status = stub_->EstopCheckIn(&context, request, &reply);

	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
		std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	return reply;
}

EstopCheckInResponse EstopClient::checkInAsync(EstopStopLevel &stopLevel, EstopEndpoint &endpoint, uint64_t challenge, uint64_t response, bool suppress_incorrect) {
	EstopCheckInRequest request;
	request.set_challenge(challenge);
	request.set_response(response);
	request.mutable_endpoint()->CopyFrom(endpoint);
	request.set_stop_level(stopLevel);

	EstopCheckInResponse reply;
	ClientContext context;
	CompletionQueue cq;
	Status status;

	std::unique_ptr<ClientAsyncResponseReader<EstopCheckInResponse>> rpc(
			stub_->PrepareAsyncEstopCheckIn(&context, request, &cq));
	rpc->StartCall();

	rpc->Finish(&reply, &status, (void *)1);
	void *got_tag;
	bool ok = false;
	GPR_ASSERT(cq.Next(&got_tag, &ok));
	GPR_ASSERT(got_tag == (void *)1);
	GPR_ASSERT(ok);

	// check status
	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
		std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	return reply;
}
