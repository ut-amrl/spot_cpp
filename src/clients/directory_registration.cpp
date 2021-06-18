#include <spot/clients/directory_registration.h>

const std::string DIRECTORY_REGISTRATION_CLIENT_NAME = "directory-registration";

DirectoryRegistrationClient::DirectoryRegistrationClient(const std::string &authority, const std::string &token) : BaseClient(DIRECTORY_REGISTRATION_CLIENT_NAME, authority, token) {}

RegisterServiceResponse DirectoryRegistrationClient::registerService(Endpoint endpoint, ServiceEntry serviceEntry){
	RegisterServiceRequest request;
	assembleRequestHeader<RegisterServiceRequest>(&request);
	request.mutable_endpoint()->CopyFrom(endpoint);
	request.mutable_service_entry()->CopyFrom(serviceEntry);
	return call<RegisterServiceRequest, RegisterServiceResponse>(request, &DirectoryRegistrationService::Stub::RegisterService);
}

UpdateServiceResponse DirectoryRegistrationClient::updateService(Endpoint endpoint, ServiceEntry serviceEntry){
	UpdateServiceRequest request;
	assembleRequestHeader<UpdateServiceRequest>(&request);
	request.mutable_endpoint()->CopyFrom(endpoint);
	request.mutable_service_entry()->CopyFrom(serviceEntry);
	return call<UpdateServiceRequest, UpdateServiceResponse>(request, &DirectoryRegistrationService::Stub::UpdateService);
}

UnregisterServiceResponse DirectoryRegistrationClient::unregisterService(std::string serviceName){
	UnregisterServiceRequest request;
	assembleRequestHeader<UnregisterServiceRequest>(&request);
	request.set_service_name(serviceName);
	return call<UnregisterServiceRequest, UnregisterServiceResponse>(request, &DirectoryRegistrationService::Stub::UnregisterService);
}