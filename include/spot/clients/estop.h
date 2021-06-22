/*
  estop.h: includes client and interface for communication with the estop service 
*/

#ifndef ESTOP_H
#define ESTOP_H

#include <spot/clients/base.h>
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
static const std::string REQUIRED_ROLE = "PDB_rooted";

/*
  class Endpoint: Represents and endpoint in the E-stop system
  // TODO: ADD TO NAMESPACE W/ OTHER CLIENTS TO DIFFERENTIATE FROM PROTOBUF CLASS
*/
class Endpoint {
public:
  Endpoint(std::shared_ptr<EstopClient> client, const std::string &name, const std::string &role, const std::string &configId, 
      const std::string &uniqueId, int64_t estopTimeout, int64_t estopCutPowerTimeout);

  /* cut(): issues a STOP command to the robot
     Input: -
     Output: -
     Side effects: -
  */
  void cut();

  /* settleThenCut(): issues SETTLE_THEN_CUT command to robot
     Input: -
     Output: -
     Side effects: -
  */
  void settleThenCut();

  /* allow(): issues NONE level command to robot
     Input: -
     Output: -
     Side effects: -
  */
  void allow();

  /* checkIn(): checks in at a specified level
     Input: level to check in at
     Output: -
     Side effects: -
  */
  void checkIn(EstopStopLevel level, bool suppressIncorrect);

  /* Mutators */
  /* setChallenge(): thread-safe write to challenge
     Input: new challenge
     Output: -
     Side effects: -
  */
  void setChallenge(uint64_t newChallenge);

  /* toProto(): translates this object into the protobuf-accepted version
     Input: -
     Output: Protobuf EstopEndpoint class
     Side effects: -
  */
  EstopEndpoint toProto();

  /* Accessors */
  const std::shared_ptr<EstopClient> getClient() const { return _client; }
  const std::string getName() const { return _name; }
  const std::string getRole() const { return _role; }
  const std::string getConfigId() const { return _configId; }
  const std::string getUniqueId() const { return _uniqueId; }

private:
  std::shared_ptr<EstopClient> _client;
  int64_t _estopTimeout;
  int64_t _estopCutPowerTimeout;
  uint64_t _challenge;
  std::string _name;
  std::string _role;
  std::string _configId;
  std::string _uniqueId;

  std::mutex _mu; // protects challenge
};

class EstopClient : public BaseClient <EstopService> { 
public:
  EstopClient(const std::string &authority, const std::string &token);

  // change
  RegisterEstopEndpointResponse registerEndpoint(const std::string &targetConfigId, EstopEndpoint &endpoint);
  RegisterEstopEndpointResponse registerEndpointAsync(const std::string &targetConfigId, EstopEndpoint &endpoint);
 
  // change
  RegisterEstopEndpointResponse replaceEndpoint(const std::string &targetConfigId, const std::string &uniqueId, EstopEndpoint &endpoint);
  RegisterEstopEndpointResponse replaceEndpointAsync(const std::string &targetConfigId, const std::string &uniqueId, EstopEndpoint &endpoint);
  
  // change
  DeregisterEstopEndpointResponse deregister(const std::string &targetConfigId, EstopEndpoint &endpoint); 
  DeregisterEstopEndpointResponse deregisterAsync(const std::string &targetConfigId, EstopEndpoint &endpoint);
  
  GetEstopConfigResponse getConfig(const std::string &targetConfigId);
  GetEstopConfigResponse getConfigAsync(const std::string &targetConfigId); 
  
  SetEstopConfigResponse setConfig(EstopConfig &config, std::string targetConfigId);
  SetEstopConfigResponse setConfigAsync(EstopConfig &config, std::string targetConfigId);
  
  GetEstopSystemStatusResponse getStatus();
  GetEstopSystemStatusResponse getStatusAsync();
  
  // change
  EstopCheckInResponse checkIn(EstopStopLevel &stopLevel, EstopEndpoint &endpoint, uint64_t challenge, uint64_t response, bool suppress_incorrect);
  EstopCheckInResponse checkInAsync(EstopStopLevel &stopLevel, EstopEndpoint &endpoint, uint64_t challenge, uint64_t response, bool suppress_incorrect);
};

/*
  class EstopThread: Does estop check-ins for a certain endpoint on a thread.
*/
class EstopThread {
public:
  EstopThread(std::shared_ptr<EstopClient> clientPtr, Endpoint &endpoint);
  ~EstopThread();

  /*  beginEstop(): begins estop check-ins on a thread
      Input: -
      Output: -
      Side effects: Creates thread for estop check-ins
  */
  void beginEstop();

  /*  endEstop(): ends estop checkins
      Input: -
      Output: -
      Side effects: Destroys thread for estop check-ins
  */
  void endEstop();

  /* Mutators */
  void setKeepRunning(bool keepRunning) { _keepRunning = keepRunning; }

  /* Accessors */
  const std::shared_ptr<EstopClient> getClient() const { return _client; }
  const std::shared_ptr<std::thread> getThread() const { return _thread; }
  const Endpoint getEndpoint() const { return _endpoint; }

private:
  /*  periodicCheckIn(): function that thread runs, loop depends on boolean set by main thread and timeout
      Input: -
      Output: -
      Side effects: issues RPCs to estop service
  */
  void periodicCheckIn();

private:
  std::shared_ptr<EstopClient> _client;
  std::shared_ptr<std::thread>_thread;
  

  Endpoint _endpoint;
  bool _keepRunning;
};

#endif
