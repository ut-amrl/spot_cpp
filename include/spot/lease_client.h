#ifndef LEASE_CLIENT_H
#define LEASE_CLIENT_H

#include "bosdyn/api/lease_service.grpc.pb.h"
#include <spot/base_client.h>

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

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using grpc::CompletionQueue;
using grpc::ClientAsyncResponseReader;
using google::protobuf::util::TimeUtil;

class LeaseClient : public BaseClient<LeaseService> {
public:
  LeaseClient(const std::string &root, const std::string &server);

  AcquireLeaseResponse acquire(const std::string &resource);
  AcquireLeaseResponse acquireAsync(const std::string &resource);
  TakeLeaseResponse take(const std::string &resource);
  TakeLeaseResponse takeAsync(const std::string &resource);
  ReturnLeaseResponse returnLease(Lease *lease);
  ReturnLeaseResponse returnLeaseAsync(Lease *lease);
  RetainLeaseResponse retainLease(Lease *lease);
  RetainLeaseResponse retainLeaseAsync(Lease *lease); 
  ListLeasesResponse listLeases(bool includeFullLeaseInfo);
  ListLeasesResponse listLeasesAsync(bool includeFullLeasesInfo);
};

#endif
