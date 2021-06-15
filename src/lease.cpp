#include <spot/lease.h>

const std::string LEASE_CLIENT_NAME = "lease";

LeaseClient::LeaseClient(const std::string &authority, const std::string &token) : BaseClient(LEASE_CLIENT_NAME, authority, token) {}

AcquireLeaseResponse LeaseClient::acquire(const std::string &resource) {
  AcquireLeaseRequest request;
  assembleRequestHeader<AcquireLeaseRequest>(&request);
  request.set_resource(resource);
  return call<AcquireLeaseRequest, AcquireLeaseResponse>(request, &LeaseService::Stub::AcquireLease); 
}

AcquireLeaseResponse LeaseClient::acquireAsync(const std::string &resource){
  AcquireLeaseRequest request;
  assembleRequestHeader<AcquireLeaseRequest>(&request);
  request.set_resource(resource);
  return callAsync<AcquireLeaseRequest, AcquireLeaseResponse>(request, &LeaseService::Stub::AsyncAcquireLease); 
}

TakeLeaseResponse LeaseClient::take(const std::string &resource){
  TakeLeaseRequest request;
  assembleRequestHeader<TakeLeaseRequest>(&request);
  request.set_resource(resource);
  return call<TakeLeaseRequest, TakeLeaseResponse>(request, &LeaseService::Stub::TakeLease);
}

TakeLeaseResponse LeaseClient::takeAsync(const std::string &resource){
  TakeLeaseRequest request;
  assembleRequestHeader<TakeLeaseRequest>(&request);
  request.set_resource(resource);
  return callAsync<TakeLeaseRequest, TakeLeaseResponse>(request, &LeaseService::Stub::AsyncTakeLease);
}

ReturnLeaseResponse LeaseClient::returnLease(Lease *lease){
  ReturnLeaseRequest request;
  assembleRequestHeader<ReturnLeaseRequest>(&request);
  request.set_allocated_lease(lease);
  return call<ReturnLeaseRequest, ReturnLeaseResponse>(request, &LeaseService::Stub::ReturnLease);
}

ReturnLeaseResponse LeaseClient::returnLeaseAsync(Lease *lease){
  ReturnLeaseRequest request;
  assembleRequestHeader<ReturnLeaseRequest>(&request);
  request.set_allocated_lease(lease);
  return callAsync<ReturnLeaseRequest, ReturnLeaseResponse>(request, &LeaseService::Stub::AsyncReturnLease);
}

RetainLeaseResponse LeaseClient::retainLease(Lease *lease){
  RetainLeaseRequest request;
  assembleRequestHeader<RetainLeaseRequest>(&request);
  request.set_allocated_lease(lease);
  return call<RetainLeaseRequest, RetainLeaseResponse>(request, &LeaseService::Stub::RetainLease);
}

RetainLeaseResponse LeaseClient::retainLeaseAsync(Lease *lease){
  RetainLeaseRequest request;
  assembleRequestHeader<RetainLeaseRequest>(&request);
  request.set_allocated_lease(lease);
  return callAsync<RetainLeaseRequest, RetainLeaseResponse>(request, &LeaseService::Stub::AsyncRetainLease);
}

ListLeasesResponse LeaseClient::listLeases(bool includeFullLeasesInfo){
  // Data we are sending to the server.
  ListLeasesRequest request;
  assembleRequestHeader<ListLeasesRequest>(&request);
  request.set_include_full_lease_info(includeFullLeasesInfo);
  return call<ListLeasesRequest, ListLeasesResponse>(request, &LeaseService::Stub::ListLeases);
}

ListLeasesResponse LeaseClient::listLeasesAsync(bool includeFullLeasesInfo){
  ListLeasesRequest request;
  assembleRequestHeader<ListLeasesRequest>(&request);
  request.set_include_full_lease_info(includeFullLeasesInfo);
  return callAsync<ListLeasesRequest, ListLeasesResponse>(request, &LeaseService::Stub::AsyncListLeases);
}

LeaseKeepAlive::LeaseKeepAlive(std::shared_ptr<LeaseClient> clientPtr, std::shared_ptr<Lease> leasePtr, int rpcIntervalSeconds) :
    _clientPtr(clientPtr),
    _leasePtr(leasePtr),
    _rpcIntervalSeconds(rpcIntervalSeconds),
    _keepRunning(true), // true for now
    _thread(&LeaseKeepAlive::periodicCheckIn, this) { // create thread
} 

LeaseKeepAlive::~LeaseKeepAlive() {
  // kill thread
  _thread.std::thread::~thread();
}

void LeaseKeepAlive::periodicCheckIn() {
  while(true) {
    checkIn(); // check into lease system
    sleep(1); // sleep for a little
  }
}

void LeaseKeepAlive::checkIn() {
  // retain the lease held in this class (change later to use lease wallet and resource)
  _clientPtr->retainLease(_leasePtr.get());
}
