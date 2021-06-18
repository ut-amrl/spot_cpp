#ifndef DIRECTORY_REGISTRATION_H
#define DIRECTORY_REGISTRATION_H

#include "bosdyn/api/directory_registration_service.grpc.pb.h"
#include <spot/clients/base.h>

using bosdyn::api::DirectoryRegistrationService;
using bosdyn::api::RegisterServiceResponse;
using bosdyn::api::RegisterServiceRequest;
using bosdyn::api::UpdateServiceResponse;
using bosdyn::api::UpdateServiceRequest;
using bosdyn::api::UnregisterServiceResponse;
using bosdyn::api::UnregisterServiceRequest;
using bosdyn::api::Endpoint;
using bosdyn::api::ServiceEntry;


const extern std::string DIRECTORY_REGISTRATION_CLIENT_NAME;

class DirectoryRegistrationClient : public BaseClient<DirectoryRegistrationService> {
public:
  DirectoryRegistrationClient(const std::string &authority, const std::string &token);

  RegisterServiceResponse registerService(Endpoint endpoint, ServiceEntry serviceEntry);
  UpdateServiceResponse updateService(Endpoint endpoint, ServiceEntry serviceEntry);
  UnregisterServiceResponse unregisterService(std::string serviceName);
};

#endif