/*
  lease.h: includes client and interface for communication with the lease service 
*/

#ifndef LEASE_H
#define LEASE_H

#include "bosdyn/api/lease_service.grpc.pb.h"
#include <spot/base.h>

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
  ReturnLeaseResponse returnLease(Lease *lease);
  ReturnLeaseResponse returnLeaseAsync(Lease *lease);
  RetainLeaseResponse retainLease(Lease lease);
  RetainLeaseResponse retainLeaseAsync(Lease *lease); 
  ListLeasesResponse listLeases(bool includeFullLeaseInfo);
  ListLeasesResponse listLeasesAsync(bool includeFullLeasesInfo);
};

// issues lease liveness checks on background thread
class LeaseKeepAlive {
public:
  LeaseKeepAlive(std::shared_ptr<LeaseClient> clientPtr, Lease lease, int rpcIntervalSeconds);
  ~LeaseKeepAlive(); // destroy thread ?

private:
  void periodicCheckIn();
  void checkIn();

private:
  std::shared_ptr<LeaseClient> _clientPtr;
  Lease _lease;
  std::thread _thread;
  std::string _resource;
  int _rpcIntervalSeconds;
  bool _keepRunning;
};

#endif
