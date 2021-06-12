#include <spot/directory.h>

DirectoryClient::DirectoryClient(const std::string &root, const std::string &server) {
  _stub = initializeNoAuthToken(server, root, "directory.spot.robot");
} 

DirectoryClient::DirectoryClient(std::string token, const std::string &root, const std::string &server) {
  _stub = initialize(server, root, token, "directory.spot.robot");
} 


ListServiceEntriesResponse DirectoryClient::list(){
  ListServiceEntriesRequest request;
  assembleRequestHeader<ListServiceEntriesRequest>(&request, "directory_client");

  return call<ListServiceEntriesRequest, ListServiceEntriesResponse>(request, &DirectoryService::Stub::ListServiceEntries);
}


ListServiceEntriesResponse DirectoryClient::listAsync(){
  ListServiceEntriesRequest request;
  assembleRequestHeader<ListServiceEntriesRequest>(&request, "directory_client");
  
  return callAsync<ListServiceEntriesRequest, ListServiceEntriesResponse>(request, &DirectoryService::Stub::AsyncListServiceEntries);
}


GetServiceEntryResponse DirectoryClient::getEntry(std::string serviceName){
  GetServiceEntryRequest request;
  assembleRequestHeader<GetServiceEntryRequest>(&request, "directory_client");
  request.set_service_name(serviceName);
  
  return call<GetServiceEntryRequest, GetServiceEntryResponse>(request, &DirectoryService::Stub::GetServiceEntry);
}


GetServiceEntryResponse DirectoryClient::getEntryAsync(std::string serviceName){
  GetServiceEntryRequest request;
  assembleRequestHeader<GetServiceEntryRequest>(&request, "directory_client");
  request.set_service_name(serviceName);
  
  return callAsync<GetServiceEntryRequest, GetServiceEntryResponse>(request, &DirectoryService::Stub::AsyncGetServiceEntry);
}