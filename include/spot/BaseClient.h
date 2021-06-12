#ifndef BASE_CLIENT_H
#define BASE_CLIENT_H

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using grpc::CompletionQueue;
using grpc::ClientAsyncResponseReader;


class SpotAuthenticator : public grpc::MetadataCredentialsPlugin {
 public:
  SpotAuthenticator(const grpc::string& ticket) : ticket_(ticket) {}

  grpc::Status GetMetadata(
      grpc::string_ref service_url, grpc::string_ref method_name,
      const grpc::AuthContext& channel_auth_context,
      std::multimap<grpc::string, grpc::string>* metadata) override {
    metadata->insert(std::make_pair("authorization", "Bearer " + ticket_));
    return grpc::Status::OK;
  }

 private:
  grpc::string ticket_;
};




template <class serv_T>
class BaseClient{
public:
	std::unique_ptr<typename serv_T::Stub> initialize(std::string hostname, std::string cert, std::string token, std::string authority);
	
	template<class req_T, class res_T>
	res_T call(req_T request, Status(serv_T::Stub::*func)(grpc::ClientContext* context, const req_T& request, res_T* response));

	template<class req_T, class res_T>
	res_T callAsync(req_T request, std::unique_ptr<ClientAsyncResponseReader<res_T>>(serv_T::Stub::*func)(grpc::ClientContext* context, const req_T& request, grpc::CompletionQueue* cq));

protected:
	std::unique_ptr<typename serv_T::Stub> _stub;
};



template <class serv_T>
std::unique_ptr<typename serv_T::Stub> BaseClient<serv_T>::initialize(std::string hostname, std::string cert, std::string token, std::string authority){
	// create options
  	grpc::SslCredentialsOptions opts;
  	opts.pem_root_certs = cert;

	// create channel arguments
  	grpc::ChannelArguments channelArgs;
  	channelArgs.SetSslTargetNameOverride(authority); // put into kv map later
    auto call_creds = grpc::MetadataCredentialsFromPlugin( std::unique_ptr<grpc::MetadataCredentialsPlugin>(new SpotAuthenticator(token)));
  	return serv_T::NewStub(grpc::CreateCustomChannel(hostname, grpc::CompositeChannelCredentials(grpc::SslCredentials(opts), call_creds), channelArgs));
}

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
		std::cout << "Success" << std::endl;
	} else {
		std::cout << status.error_code() << ": " << status.error_message()
				<< std::endl;
	}

	return reply;
}

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
	if (status.ok()) {
		std::cout << "Success" << std::endl;
	} else {
		std::cout << status.error_code() << ": " << status.error_message()
				<< std::endl;
	}

	return reply;
}



#endif