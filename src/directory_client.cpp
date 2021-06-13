#include <spot/directory_client.h>

DirectoryClient::DirectoryClient(const std::string &root, const std::string &server) {
  _stub = initializeNoAuthToken(server, root, "directory.spot.robot");
  _clientName = "directory";
}  

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