/*
  base.h: includes base client of which all other clients and interfaces inherit from 
*/

#ifndef BASE_H
#define BASE_H

#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <thread>

#include <spot/common.h>

#include <spot/exception.h>
#include <spot/utils.h>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using grpc::CompletionQueue;
using grpc::ClientAsyncResponseReader;
using google::protobuf::util::TimeUtil;

using grpc::StatusCode;

/* EXCEPTIONS */
template <class request_T>
class UnauthenticatedError : RpcError<request_T> {
public:
    UnauthenticatedError(const request_T &request, const std::string &message) : RpcError<request_T>(request, message) {}
};

template <class request_T>
class PermissionDeniedError : RpcError<request_T> {
public:
	PermissionDeniedError(const request_T &request, const std::string &message) : RpcError<request_T>(request, message) {}
};

template <class request_T>
class InvalidArgumentError : RpcError<request_T> {
public:
	InvalidArgumentError(const request_T &request, const std::string &message) : RpcError<request_T>(request, message) {}
};

/* SOURCE */
template <class serv_T>
class BaseClient {
public:
	BaseClient(const std::string &clientName, const std::string &authority, const std::string &token);	
	std::string getClientName() const {return _clientName;}

protected:
	template<class req_T>
	void assembleRequestHeader(req_T* req);

	template<class req_T, class res_T>
	res_T call(req_T request, Status(serv_T::Stub::*func)(grpc::ClientContext* context, const req_T& request, res_T* response));

	template<class req_T, class res_T>
	res_T callAsync(req_T request, std::unique_ptr<ClientAsyncResponseReader<res_T>>(serv_T::Stub::*func)(grpc::ClientContext* context, const req_T& request, grpc::CompletionQueue* cq));

	std::string getAuthority() const {return _authority;}

protected:
	std::unique_ptr<typename serv_T::Stub> _stub;
	std::string _authority;
	std::string _clientName;

private:
	class SpotAuthenticator : public grpc::MetadataCredentialsPlugin {
	public:
	SpotAuthenticator(const grpc::string& ticket) : ticket_(ticket) {}

	grpc::Status GetMetadata(
		grpc::string_ref service_url, grpc::string_ref method_name,
		const grpc::AuthContext& channel_auth_context,
		std::multimap<grpc::string, grpc::string>* metadata) override {
		metadata->insert(std::make_pair(_authorization_key, _authorization_value_prefix + ticket_));
		return grpc::Status::OK;
	}

	private:
	grpc::string ticket_;
	std::string _authorization_key = "authorization";
	std::string _authorization_value_prefix = "Bearer ";
	};
};

// TypedClient(): constructor for TypedClient, takes in clientName, authority, and auth token
template <class serv_T>
BaseClient<serv_T>::BaseClient(const std::string &clientName, const std::string &authority, const std::string &token) : 
		_clientName(clientName),
		_authority(authority) {
	
	// get hostname
	std::string hostName = DEFAULT_SPOT_SERVER + DEFAULT_SECURE_PORT;
	
	// create options
	grpc::SslCredentialsOptions opts;
	opts.pem_root_certs = DEFAULT_ROOT_CERT;

	// create channel arguments
	grpc::ChannelArguments channelArgs;
	channelArgs.SetSslTargetNameOverride(authority);

	// get call credentials (should work even if token is empty)
	auto call_creds = grpc::MetadataCredentialsFromPlugin(std::unique_ptr<grpc::MetadataCredentialsPlugin>(new SpotAuthenticator(token)));
	_stub = serv_T::NewStub(grpc::CreateCustomChannel(hostName, grpc::CompositeChannelCredentials(grpc::SslCredentials(opts), call_creds), channelArgs));
}

// call(): performs rpc
template<class serv_T>
template<class req_T, class res_T>
res_T BaseClient<serv_T>::call(req_T request, Status(serv_T::Stub::*func)(grpc::ClientContext* context, const req_T& request, res_T* response)){
	// Container for the data we expect from the server.
	res_T reply;

	// Context for the client. It could be used to convey extra information to
	// the server and/or tweak certain RPC behaviors.
	ClientContext context;

	// The actual RPC.
	Status status = (*(_stub).*func)(&context, request, &reply);

	// Act upon its status.
	if (status.ok()) {
		//std::cout << "Success" << std::endl;
	} else {
		std::cout << status.error_code() << ": " << status.error_message()
				<< std::endl;
	}

	if (!status.ok()) {
		//  switch based on error case
		switch (status.error_code()) {
			case grpc::StatusCode::CANCELLED:
				break;
			case grpc::StatusCode::INVALID_ARGUMENT:
				throw InvalidArgumentError<req_T>(request, "Invalid argument(s) for RPC call.");
				break;
			case grpc::StatusCode::UNKNOWN:
				break;
			case grpc::StatusCode::DEADLINE_EXCEEDED:
				break;
			case grpc::StatusCode::NOT_FOUND:
				break;
			case grpc::StatusCode::ALREADY_EXISTS:
				break;
			case grpc::StatusCode::PERMISSION_DENIED:
				throw PermissionDeniedError<req_T>(request, "Permission denied.");
				break;
			case grpc::StatusCode::UNAUTHENTICATED:
				throw UnauthenticatedError<req_T>(request, "Not authenticated.");
				break;
			default:
				;
		}
	}

	return reply;
}

// callAsync(): performs async rpc
template<class serv_T>
template<class req_T, class res_T>
res_T BaseClient<serv_T>::callAsync(req_T request, std::unique_ptr<ClientAsyncResponseReader<res_T>>(serv_T::Stub::*func)(grpc::ClientContext* context, const req_T& request, grpc::CompletionQueue* cq)){
	// Container for the data we expect from the server.
	res_T reply;

	// Context for the client. It could be used to convey extra information to
	// the server and/or tweak certain RPC behaviors.
	ClientContext context;

	// The actual RPC.
	CompletionQueue cq;
	std::unique_ptr<ClientAsyncResponseReader<res_T>> rpc((*(_stub).*func)(&context, request, &cq));

	// status
	Status status;
	rpc->Finish(&reply, &status, (void*)1);

	// Act upon its status.
	if (!status.ok()) {
		//  switch based on error case
		switch (status.error_code()) {
			case grpc::StatusCode::CANCELLED:
				break;
			case grpc::StatusCode::INVALID_ARGUMENT:
				break;
			case grpc::StatusCode::UNKNOWN:
				break;
			case grpc::StatusCode::DEADLINE_EXCEEDED:
				break;
			case grpc::StatusCode::NOT_FOUND:
				break;
			case grpc::StatusCode::ALREADY_EXISTS:
				break;
			case grpc::StatusCode::PERMISSION_DENIED:
				throw PermissionDeniedError<req_T>(request, "Permission denied.");
				break;
			case grpc::StatusCode::UNAUTHENTICATED:
				throw UnauthenticatedError<req_T>(request, "Not authenticated.");
				break;
			default:
				;
		}
	}

	return reply;
}

// assembleRequestHeader(): fills in header info for requests
template<class serv_T>
template<class req_T>
void BaseClient<serv_T>::assembleRequestHeader(req_T* req){
	req->mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  	req->mutable_header()->set_client_name(_clientName);
}

#endif