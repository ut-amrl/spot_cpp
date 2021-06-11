#include <spot/estop.h>

EstopClient::EstopClient(const std::string &root, const std::string &server) {
	// create options
  	grpc::SslCredentialsOptions opts;
  	opts.pem_root_certs = root;

	// create channel arguments
  	grpc::ChannelArguments channelArgs;
  	channelArgs.SetSslTargetNameOverride("estop.spot.robot"); // put into kv map later
  	stub_ = EstopService::NewStub(grpc::CreateCustomChannel(server, grpc::SslCredentials(opts), channelArgs));
}

 
RegisterEstopEndpointResponse EstopClient::registerEndpoint(const std::string &targetConfigId, EstopEndpoint &endpoint) {
	RegisterEstopEndpointRequest request;
	request.mutable_new_endpoint()->CopyFrom(endpoint);
	request.mutable_header()->set_client_name("estop_client");

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
	request.mutable_header()->set_client_name("estop_client");

	
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
	request.mutable_header()->set_client_name("estop_client");

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
	request.mutable_header()->set_client_name("estop_client");


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
	request.mutable_header()->set_client_name("estop_client");

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
	request.mutable_header()->set_client_name("estop_client");

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
	request.mutable_header()->set_client_name("estop_client");


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
	request.mutable_header()->set_client_name("estop_client");

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
	request.mutable_header()->set_client_name("estop_client");

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
	request.mutable_header()->set_client_name("estop_client");

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
	request.mutable_header()->set_client_name("estop_client");

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
	request.mutable_header()->set_client_name("estop_client");


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
