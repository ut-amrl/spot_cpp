#include <spot_comm/lease.h>

Lease::Lease(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) {
  grpc::SslCredentialsOptions opts = {root, key, cert};
  stub_ = LeaseService::NewStub(grpc::CreateChannel(server, grpc::SslCredentials(opts)));
}

// Assembles the client's payload, sends it and presents the response back
// from the server.
AcquireLeaseResponse Lease::AcquireLease(const std::string& resource) {
  // Data we are sending to the server.
  AcquireLeaseRequest request;
  request.set_resource(resource);

  // Container for the data we expect from the server.
  AcquireLeaseResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->AcquireLease(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << reply.status() << ", Lease Acquired: " << reply.lease().resource() <<
    ", New Owner: " << reply.lease_owner().user_name() << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
  return reply;
}

TakeLeaseResponse Lease::TakeLease(const std::string& resource) {
  // Data we are sending to the server.
  TakeLeaseRequest request;
  request.set_resource(resource);

  // Container for the data we expect from the server.
  TakeLeaseResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->TakeLease(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << reply.status() << ", Lease Taken: " << reply.lease().resource() <<
    ", New Owner: " << reply.lease_owner().user_name() << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
  return reply;
}

ReturnLeaseResponse Lease::ReturnLease(Lease* lease) {
  // Data we are sending to the server.
  ReturnLeaseRequest request;
  request.set_allocated_lease(lease);

  // Container for the data we expect from the server.
  ReturnLeaseResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->ReturnLease(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << reply.status() << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
  return reply;
}

ListLeasesResponse Lease::ListLeases() {
  // Data we are sending to the server.
  ListLeasesRequest request;

  // Container for the data we expect from the server.
  ListLeasesResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->ListLeases(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << status.ok() << std::endl << "1st Lease: " << reply.resources(0).resource() << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
  return reply;
}

RetainLeaseResponse Lease::RetainLease() {
  // Data we are sending to the server.
  RetainLeaseRequest request;

  // Container for the data we expect from the server.
  RetainLeaseResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->RetainLease(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << status.ok() << " Lease Retained: " << reply.lease_use_result().attempted_lease().resource() << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
  return reply;
}

// new 

AcquireLeaseResponse Lease::acquire(const std::string &resource) {
  // Data we are sending to the server.
  AcquireLeaseRequest request;
  request.set_resource(resource);

  // Container for the data we expect from the server.
  AcquireLeaseResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->AcquireLease(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << reply.status() << ", Lease Acquired: " << reply.lease().resource() <<
    ", New Owner: " << reply.lease_owner().user_name() << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
  return reply;  
}

AcquireLeaseResponse Lease::acquireAsync(const std::string &resource){
    // Data we are sending to the server.
  AcquireLeaseRequest request;
  request.set_resource(resource);

  // Container for the data we expect from the server.
  AcquireLeaseResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<AcquireLeaseResponse>> rpc(stub_->AsyncAcquireLease(&context, request, &cq));

  // status
  Status status;
  rpc->Finish(&reply, &status, (void*)1);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << reply.status() << ", Lease Acquired: " << reply.lease().resource() <<
    ", New Owner: " << reply.lease_owner().user_name() << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
  return reply;
}

TakeLeaseResponse Lease::take(const std::string &resource){
  // Data we are sending to the server.
  TakeLeaseRequest request;
  request.set_resource(resource);

  // Container for the data we expect from the server.
  TakeLeaseResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->TakeLease(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << reply.status() << ", Lease Acquired: " << reply.lease().resource() <<
    ", New Owner: " << reply.lease_owner().user_name() << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
  return reply;
}

TakeLeaseResponse Lease::takeAsync(const std::string &resource){
  // Data we are sending to the server.
  TakeLeaseRequest request;
  request.set_resource(resource);

  // Container for the data we expect from the server.
  TakeLeaseResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<TakeLeaseResponse>> rpc(stub_->AsyncTakeLease(&context, request, &cq));

  // status
  Status status;
  rpc->Finish(&reply, &status, (void*)1);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << reply.status() << ", Lease Acquired: " << reply.lease().resource() <<
    ", New Owner: " << reply.lease_owner().user_name() << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
  return reply;
}

ReturnLeaseResponse Lease::returnLease(Lease* lease){
  // Data we are sending to the server.
  ReturnLeaseRequest request;
  request.set_allocated_lease(lease);

  // Container for the data we expect from the server.
  ReturnLeaseResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->ReturnLease(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << reply.status() << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
  return reply;
}

ReturnLeaseResponse Lease::returnLeaseAsync(Lease* lease){
  // Data we are sending to the server.
  ReturnLeaseRequest request;
  request.set_allocated_lease(lease);

  // Container for the data we expect from the server.
  ReturnLeaseResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<ReturnLeaseResponse>> rpc(stub_->AsyncReturnLease(&context, request, &cq));

  // status
  Status status;
  rpc->Finish(&reply, &status, (void*)1);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << reply.status() << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
  return reply;
}

RetainLeaseResponse Lease::retainLease(Lease &lease){
  // Data we are sending to the server.
  RetainLeaseRequest request;

  // Container for the data we expect from the server.
  RetainLeaseResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->RetainLease(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << status.ok() << " Lease Retained: " << reply.lease_use_result().attempted_lease().resource() << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
  return reply;
}

RetainLeaseResponse Lease::retainLeaseAsync(Lease &lease){
    // Data we are sending to the server.
  RetainLeaseRequest request;

  // Container for the data we expect from the server.
  RetainLeaseResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<RetainLeaseResponse>> rpc(stub_->AsyncRetainLease(&context, request, &cq));

  // status
  Status status;
  rpc->Finish(&reply, &status, (void*)1);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << status.ok() << " Lease Retained: " << reply.lease_use_result().attempted_lease().resource() << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
  return reply;
}

ListLeasesResponse Lease::listLeases(bool includeFullLeaseInfo){
  // Data we are sending to the server.
  ListLeasesRequest request;

  // Container for the data we expect from the server.
  ListLeasesResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->ListLeases(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << status.ok() << std::endl << "1st Lease: " << reply.resources(0).resource() << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
  return reply;	
}

ListLeasesResponse Lease::listLeasesAsync(bool includeFullLeasesInfo){
  // Data we are sending to the server.
  ListLeasesRequest request;

  // Container for the data we expect from the server.
  ListLeasesResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<ListLeasesResponse>> rpc(stub_->AsyncListLeases(&context, request, &cq));

  // status
  Status status;
  rpc->Finish(&reply, &status, (void*)1);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Status: " << status.ok() << std::endl << "1st Lease: " << reply.resources(0).resource() << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
  return reply;
}