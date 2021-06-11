#include <spot/auth.h>

AuthClient::AuthClient(const std::string &root, const std::string &server) {
	// create options
  	grpc::SslCredentialsOptions opts;
  	opts.pem_root_certs = root;

	// create channel arguments
  	grpc::ChannelArguments channelArgs;
  	channelArgs.SetSslTargetNameOverride("auth.spot.robot"); // put into kv map later
  	stub_ = AuthService::NewStub(grpc::CreateCustomChannel(server, grpc::SslCredentials(opts), channelArgs));
}

GetAuthTokenResponse AuthClient::auth(const std::string &user, const std::string &pass) {
	// populate request
	GetAuthTokenRequest request;
	request.set_username(user);
	request.set_password(pass);
	request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
	request.mutable_header()->set_client_name("auth_client");

	GetAuthTokenResponse response;
	ClientContext context;
	Status status = stub_->GetAuthToken(&context, request, &response);

	if (status.ok()) {
	  std::cout << "[SUCCEEDED]" << std::endl;
	  std::cout << "Token Status: " << response.status() << ", Token: " << response.token() << std::endl;
	} else {
          std::cout << "[FAILED]" << std::endl;
	  std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	return response;
}

GetAuthTokenResponse AuthClient::authAsync(const std::string &user, const std::string &pass) {
	GetAuthTokenRequest request;
	request.set_username(user);
	request.set_password(pass);
	request.mutable_header()->set_client_name("auth_client");

	GetAuthTokenResponse response;
	ClientContext context;

	// queue for asynchronous communication
	CompletionQueue cq;

	// storage for status
	Status status;

	// creates rpc object, but no call yet
	std::unique_ptr<ClientAsyncResponseReader<GetAuthTokenResponse>> rpc(
			stub_->PrepareAsyncGetAuthToken(&context, request, &cq));

	// initiates rpc call 
	rpc->StartCall();

	// update response, get status
	rpc->Finish(&response, &status, (void *)1);
	void *got_tag;
	bool ok = false;

	// block until next result is available
	GPR_ASSERT(cq.Next(&got_tag, &ok));

	// verify result corresponds to previous request and request completed successfully
	GPR_ASSERT(got_tag == (void *)1);
	GPR_ASSERT(ok);

	// act upon rpc status
	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
		std::cout << "Token Status: " << response.status() << ", Token: " << response.token() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	// return response
	return response;
}

GetAuthTokenResponse AuthClient::authWithToken(const std::string &token) {
	GetAuthTokenRequest request;
	request.set_token(token);
	request.mutable_header()->set_client_name("auth_client");

	GetAuthTokenResponse response;
	ClientContext context;
	Status status = stub_->GetAuthToken(&context, request, &response);

	if (status.ok()) {
	  std::cout << "[SUCCEEDED]" << std::endl;
	  std::cout << "Token Status: " << response.status() << ", Token: " << response.token() << std::endl;
	} else {
          std::cout << "[FAILED]" << std::endl;
	  std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	return response;
}

GetAuthTokenResponse AuthClient::authWithTokenAsync(const std::string &token) {
	GetAuthTokenRequest request;
	request.set_token(token);
	request.mutable_header()->set_client_name("auth_client");

	GetAuthTokenResponse response;
	ClientContext context;
	CompletionQueue cq;
	Status status;

	std::unique_ptr<ClientAsyncResponseReader<GetAuthTokenResponse>> rpc(
			stub_->PrepareAsyncGetAuthToken(&context, request, &cq));
	rpc->StartCall();

	rpc->Finish(&response, &status, (void *)1);
	void *got_tag;
	bool ok = false;

	GPR_ASSERT(cq.Next(&got_tag, &ok));
	GPR_ASSERT(got_tag == (void *)1);
	GPR_ASSERT(ok);

	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
		std::cout << "Token Status: " << response.status() << ", Token: " << response.token() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}

	return response;
}
