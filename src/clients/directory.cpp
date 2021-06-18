#include <spot/clients/directory.h>

const std::string DIRECTORY_CLIENT_NAME = "directory";
const static std::string AUTHORITY = "api.spot.robot";

DirectoryClient::DirectoryClient(const std::string &token) : BaseClient(DIRECTORY_CLIENT_NAME, AUTHORITY, token) {}  

ListServiceEntriesResponse DirectoryClient::list(){
    ListServiceEntriesRequest request;
    assembleRequestHeader<ListServiceEntriesRequest>(&request);
    return call<ListServiceEntriesRequest, ListServiceEntriesResponse>(request, &DirectoryService::Stub::ListServiceEntries);
}

ListServiceEntriesResponse DirectoryClient::listAsync(){
  ListServiceEntriesRequest request;
  assembleRequestHeader<ListServiceEntriesRequest>(&request);
  return callAsync<ListServiceEntriesRequest, ListServiceEntriesResponse>(request, &DirectoryService::Stub::AsyncListServiceEntries);
}

GetServiceEntryResponse DirectoryClient::getEntry(std::string serviceName){
  GetServiceEntryRequest request;
  assembleRequestHeader<GetServiceEntryRequest>(&request);
  request.set_service_name(serviceName);
  return call<GetServiceEntryRequest, GetServiceEntryResponse>(request, &DirectoryService::Stub::GetServiceEntry);
}

GetServiceEntryResponse DirectoryClient::getEntryAsync(std::string serviceName){
  GetServiceEntryRequest request;
  assembleRequestHeader<GetServiceEntryRequest>(&request);
  request.set_service_name(serviceName);
  return callAsync<GetServiceEntryRequest, GetServiceEntryResponse>(request, &DirectoryService::Stub::AsyncGetServiceEntry);
}