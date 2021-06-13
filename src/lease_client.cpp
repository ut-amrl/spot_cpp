#include <spot/lease_client.h>

LeaseClient::LeaseClient(const std::string &root, const std::string &server) {
	_stub = initializeNoAuthToken(server, root, "");
  _clientName = "lease";
}

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
