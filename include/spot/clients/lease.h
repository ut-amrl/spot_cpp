/*
  lease.h: includes client and interface for communication with the lease service 
*/

#ifndef LEASE_H
#define LEASE_H

#include <spot/clients/base.h>
#include "bosdyn/api/lease_service.grpc.pb.h"


using bosdyn::api::AcquireLeaseRequest;
using bosdyn::api::AcquireLeaseResponse;
using bosdyn::api::Lease;
using bosdyn::api::LeaseOwner;
using bosdyn::api::LeaseResource;
using bosdyn::api::LeaseUseResult;
using bosdyn::api::ListLeasesRequest;
using bosdyn::api::ListLeasesResponse;
using bosdyn::api::RetainLeaseRequest;
using bosdyn::api::RetainLeaseResponse;
using bosdyn::api::ReturnLeaseRequest;
using bosdyn::api::ReturnLeaseResponse;
using bosdyn::api::TakeLeaseRequest;
using bosdyn::api::TakeLeaseResponse;
using bosdyn::api::LeaseService;

const extern std::string LEASE_CLIENT_NAME;

class LeaseClient : public BaseClient<LeaseService> {
public:
  LeaseClient(const std::string &authority, const std::string &token);

  AcquireLeaseResponse acquire(const std::string &resource);
  AcquireLeaseResponse acquireAsync(const std::string &resource);
  TakeLeaseResponse take(const std::string &resource);
  TakeLeaseResponse takeAsync(const std::string &resource);
  ReturnLeaseResponse returnLease(Lease lease);
  ReturnLeaseResponse returnLeaseAsync(Lease lease);
  RetainLeaseResponse retainLease(Lease lease);
  RetainLeaseResponse retainLeaseAsync(Lease lease); 
  ListLeasesResponse listLeases(bool includeFullLeaseInfo);
  ListLeasesResponse listLeasesAsync(bool includeFullLeasesInfo);
};

/*
  class LeaseThread: issues lease check-ins on a thread
*/
class LeaseThread {
  
public:
  static int DEFAULT_RPC_INTERVAL_SECS = 2;

  LeaseThread(std::shared_ptr<LeaseClient> clientPtr, Lease lease);
  ~LeaseThread();

  /*  beginEstop(): begins lease check-ins on a thread
      Input: -
      Output: -
      Side effects: Creates thread for lease check-ins
  */
  void beginLease();

  /*  endEstop(): ends lease checkins
      Input: -
      Output: -
      Side effects: Destroys thread for estop check-ins
  */
  void endLease();

  /* Mutators */
  void setKeepRunning(bool keepRunning) { _keepRunning = keepRunning; }
  
  /* Accessors */
  const std::shared_ptr<std::thread> getThread() const { return _thread; }
  const Lease getLease() const { return _lease; }

private:
  /* periodicCheckIn(): function that thread runs, loop depends on boolean set by main thread
     Input: -
     Output: -
     Side effects: issues RPCs to lease service
  */
  void periodicCheckIn();

private:
  std::shared_ptr<LeaseClient> _client;
  std::shared_ptr<std::thread> _thread;

  Lease _lease; // copy of lease
  bool _keepRunning;
};

#endif
