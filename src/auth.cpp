#include <spot/auth.h>

AuthClient::AuthClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) {
  grpc::SslCredentialsOptions opts = {root, key, cert};
  stub_ = AuthService::NewStub(grpc::CreateChannel(server, grpc::SslCredentials(opts)));
}

std::string AuthClient::GetAuthToken(const std::string& user, const std::string& pass) {
  // Data we are sending to the server.
  GetAuthTokenRequest request;
  request.set_username(user);
  request.set_password(pass);
  // request.set_application_token(appToken); (deprecated)

  // Container for the data we expect from the server.
  GetAuthTokenResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->GetAuthToken(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Token Status: " << reply.status() << ", Token: " << reply.token() << std::endl;
    return reply.token();
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    return "RPC failed";
  }
}

GetAuthTokenResponse AuthClient::auth(const std::string &user, const std::string &pass) {
	// populate request
	GetAuthTokenRequest request;
	request.set_username(user);
	request.set_password(pass);

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
