#ifndef DIRECTORY_CLIENT_H
#define DIRECTORY_CLIENT_H

#include <spot/base_client.h>
#include "bosdyn/api/directory_service.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using grpc::CompletionQueue;
using grpc::ClientAsyncResponseReader;

using bosdyn::api::DirectoryService;
using bosdyn::api::ListServiceEntriesResponse;
using bosdyn::api::ListServiceEntriesRequest;
using bosdyn::api::GetServiceEntryRequest;
using bosdyn::api::GetServiceEntryResponse;

using google::protobuf::util::TimeUtil;

class DirectoryClient : public BaseClient<DirectoryService> {
public:
	DirectoryClient(const std::string &root, const std::string &server);

	ListServiceEntriesResponse list();
	ListServiceEntriesResponse listAsync();
	GetServiceEntryResponse getEntry(std::string serviceName);
	GetServiceEntryResponse getEntryAsync(std::string serviceName);
};


#endif