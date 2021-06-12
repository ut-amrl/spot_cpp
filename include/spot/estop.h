#ifndef ESTOP_H
#define ESTOP_H

#include <iostream>
#include <memory>
#include <string>

#include <spot/BaseClient.h>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/estop_service.grpc.pb.h"

using bosdyn::api::RegisterEstopEndpointRequest;
using bosdyn::api::RegisterEstopEndpointResponse;
using bosdyn::api::DeregisterEstopEndpointRequest;
using bosdyn::api::DeregisterEstopEndpointResponse;
using bosdyn::api::EstopCheckInRequest;
using bosdyn::api::EstopCheckInResponse;
using bosdyn::api::GetEstopConfigRequest;
using bosdyn::api::GetEstopConfigResponse;
using bosdyn::api::SetEstopConfigRequest;
using bosdyn::api::SetEstopConfigResponse;
using bosdyn::api::GetEstopSystemStatusRequest;
using bosdyn::api::GetEstopSystemStatusResponse;
using bosdyn::api::EstopEndpoint;
using bosdyn::api::EstopService;
using bosdyn::api::EstopConfig;
using bosdyn::api::EstopSystemStatus;
using bosdyn::api::EstopStopLevel;
using google::protobuf::Duration;

using grpc::Channel;
using grpc::CompletionQueue;
using grpc::ClientAsyncResponseReader;
using grpc::ClientContext;
using grpc::Status;

class EstopClient : public BaseClient <EstopService> {
public:
  EstopClient(const std::string &root, const std::string &server);

  RegisterEstopEndpointResponse registerEndpoint(const std::string &targetConfigId, EstopEndpoint &endpoint);
  RegisterEstopEndpointResponse registerEndpointAsync(const std::string &targetConfigId, EstopEndpoint &endpoint);
  DeregisterEstopEndpointResponse deregister(const std::string &targetConfigId, EstopEndpoint &endpoint); 
  DeregisterEstopEndpointResponse deregisterAsync(const std::string &targetConfigId, EstopEndpoint &endpoint);
  GetEstopConfigResponse getConfig(const std::string &targetConfigId);
  GetEstopConfigResponse getConfigAsync(const std::string &targetConfigId); 
  SetEstopConfigResponse setConfig(EstopConfig &config, std::string targetConfigId);
  SetEstopConfigResponse setConfigAsync(EstopConfig &config, std::string targetConfigId);
  GetEstopSystemStatusResponse getStatus();
  GetEstopSystemStatusResponse getStatusAsync();
  EstopCheckInResponse checkIn(EstopStopLevel &stopLevel, EstopEndpoint &endpoint, uint64_t challenge, uint64_t response, bool suppress_incorrect);
  EstopCheckInResponse checkInAsync(EstopStopLevel &stopLevel, EstopEndpoint &endpoint, uint64_t challenge, uint64_t response, bool suppress_incorrect);
};

#endif
