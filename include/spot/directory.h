#ifndef DIRECTORY_H
#define DIRECTORY_H

#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include <spot/BaseClient.h>

#include "bosdyn/api/directory_service.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"
#include <google/protobuf/util/time_util.h>

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

class DirectoryClient : BaseClient <DirectoryService>{
public:
	DirectoryClient(const std::string &root, const std::string &server);
  	DirectoryClient(std::string token, const std::string &root, const std::string &server);

	ListServiceEntriesResponse list();
	ListServiceEntriesResponse listAsync();
	GetServiceEntryResponse getEntry(std::string serviceName);
	GetServiceEntryResponse getEntryAsync(std::string serviceName);

};


#endif