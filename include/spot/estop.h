/*
  estop.h: includes client and interface for communication with the estop service 
*/

#ifndef ESTOP_H
#define ESTOP_H

#include <spot/base.h>
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

const extern std::string ESTOP_CLIENT_NAME;
const std::string REQUIRED_ROLE = "PDB_rooted";

typedef bool (*keepAliveCallback)(void);

class EstopClient : public BaseClient <EstopService> {
public:
  EstopClient(const std::string &authority, const std::string &token);

  RegisterEstopEndpointResponse registerEndpoint(const std::string &targetConfigId, EstopEndpoint &targetEndpoint, EstopEndpoint &endpoint);
  RegisterEstopEndpointResponse registerEndpointAsync(const std::string &targetConfigId, EstopEndpoint &endpoint);
  DeregisterEstopEndpointResponse deregister(const std::string &targetConfigId, EstopEndpoint &endpoint); 
  DeregisterEstopEndpointResponse deregisterAsync(const std::string &targetConfigId, EstopEndpoint &endpoint);
  GetEstopConfigResponse getConfig(const std::string &targetConfigId);
  GetEstopConfigResponse getConfigAsync(const std::string &targetConfigId); 
  SetEstopConfigResponse setConfig(EstopConfig &config);
  SetEstopConfigResponse setConfig(EstopConfig &config, std::string targetConfigId);
  SetEstopConfigResponse setConfigAsync(EstopConfig &config, std::string targetConfigId);
  GetEstopSystemStatusResponse getStatus();
  GetEstopSystemStatusResponse getStatusAsync();
  EstopCheckInResponse checkIn(EstopStopLevel &stopLevel, EstopEndpoint &endpoint, uint64_t challenge, uint64_t response, bool suppress_incorrect);
  EstopCheckInResponse checkInAsync(EstopStopLevel &stopLevel, EstopEndpoint &endpoint, uint64_t challenge, uint64_t response, bool suppress_incorrect);
};

// does periodic estop check-ins on a thread
class EstopKeepAlive {
public:
  EstopKeepAlive(std::shared_ptr<EstopClient> clientPtr, EstopEndpoint endpoint, EstopStopLevel stopLevel, int rpcTimeoutSeconds, int rpcIntervalSeconds, uint64_t challenge /*, keepAliveCallback cbFunc*/);
  ~EstopKeepAlive(); // destroy thread ?

private:
  void periodicCheckIn();
  void checkIn();

private:
  std::shared_ptr<EstopClient> _clientPtr;
  EstopEndpoint _endpoint;
  EstopStopLevel _stopLevel;
  std::thread _thread;

  int _rpcTimeoutSeconds;
  int _rpcIntervalSeconds;
  uint64_t _challenge;
  bool _keepRunning;
};

#endif
