/*
  directory.h: includes client and interface for communication with the directory service 
*/

#ifndef DIRECTORY_H
#define DIRECTORY_H

#include <spot/base.h>
#include "bosdyn/api/directory_service.grpc.pb.h"

using bosdyn::api::DirectoryService;
using bosdyn::api::ListServiceEntriesResponse;
using bosdyn::api::ListServiceEntriesRequest;
using bosdyn::api::GetServiceEntryRequest;
using bosdyn::api::GetServiceEntryResponse;

class DirectoryClient : public BaseClient<DirectoryService> {
public:
	DirectoryClient(const std::string &token);

	ListServiceEntriesResponse list();
	ListServiceEntriesResponse listAsync();
	GetServiceEntryResponse getEntry(std::string serviceName);
	GetServiceEntryResponse getEntryAsync(std::string serviceName);
};


#endif