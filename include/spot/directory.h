/*
  directory.h: includes client for communication with the directory service 
*/

#ifndef DIRECTORY_H
#define DIRECTORY_H

#include <spot/base.h>
#include "bosdyn/api/directory_service.grpc.pb.h"

#include <list>

using bosdyn::api::DirectoryService;
using bosdyn::api::ListServiceEntriesResponse;
using bosdyn::api::ListServiceEntriesRequest;
using bosdyn::api::GetServiceEntryRequest;
using bosdyn::api::GetServiceEntryResponse;

const extern std::string DIRECTORY_CLIENT_NAME;

class ServiceEntry {
public:
private:
};

class DirectoryClient : public BaseClient<DirectoryService> {
public:
	DirectoryClient(const std::string &token);

	// std::list<ServiceEntry> listServiceEntries(bool async); // idea for containing grpc calls

	ListServiceEntriesResponse list();
	ListServiceEntriesResponse listAsync();

	// TODO: figure out way to interface these methods which return gRPC objects
	GetServiceEntryResponse getEntry(std::string serviceName);
	GetServiceEntryResponse getEntryAsync(std::string serviceName);
};


#endif