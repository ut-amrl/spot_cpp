#include <spot/clients/lease.h>

namespace ClientLayer {

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

  ReturnLeaseResponse LeaseClient::returnLease(bosdyn::api::Lease lease){
    ReturnLeaseRequest request;
    assembleRequestHeader<ReturnLeaseRequest>(&request);
    request.set_allocated_lease(&lease);
    return call<ReturnLeaseRequest, ReturnLeaseResponse>(request, &LeaseService::Stub::ReturnLease);
  }

  ReturnLeaseResponse LeaseClient::returnLeaseAsync(bosdyn::api::Lease lease){
    ReturnLeaseRequest request;
    assembleRequestHeader<ReturnLeaseRequest>(&request);
    request.set_allocated_lease(&lease);
    return callAsync<ReturnLeaseRequest, ReturnLeaseResponse>(request, &LeaseService::Stub::AsyncReturnLease);
  }

  RetainLeaseResponse LeaseClient::retainLease(bosdyn::api::Lease lease){
    RetainLeaseRequest request;
    assembleRequestHeader<RetainLeaseRequest>(&request);
    request.mutable_lease()->CopyFrom(lease);
    return call<RetainLeaseRequest, RetainLeaseResponse>(request, &LeaseService::Stub::RetainLease);
  }

  RetainLeaseResponse LeaseClient::retainLeaseAsync(bosdyn::api::Lease lease){
    RetainLeaseRequest request;
    assembleRequestHeader<RetainLeaseRequest>(&request);
    request.set_allocated_lease(&lease);
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

  int LeaseThread::DEFAULT_RPC_INTERVAL_SECS = 1;

  LeaseThread::LeaseThread(std::shared_ptr<LeaseClient> clientPtr, bosdyn::api::Lease lease) :
      _client(clientPtr),
      _lease(lease) {}

  LeaseThread::~LeaseThread() {
    endLease();
  }

  void LeaseThread::beginLease() {
    _keepRunning = true;

    // just begin thread
    _thread = std::shared_ptr<std::thread>(new std::thread(&LeaseThread::periodicCheckIn, this));
  }

  void LeaseThread::endLease() {
    _keepRunning = false;
    if (_thread->joinable()) {
      _thread->join();
    }
  }

  void LeaseThread::periodicCheckIn() {
    while(_keepRunning) {
      _client->retainLease(_lease);
      std::this_thread::sleep_for(std::chrono::seconds(DEFAULT_RPC_INTERVAL_SECS));
    }
  }

};
