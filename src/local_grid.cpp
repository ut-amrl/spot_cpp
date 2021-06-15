#include <spot/local_grid.h>

const std::string LOCAL_GRID_CLIENT_NAME = "local-grid";

LocalGridClient::LocalGridClient(const std::string &authority, const std::string &token) : BaseClient(LOCAL_GRID_CLIENT_NAME, authority, token) {}

GetLocalGridTypesResponse LocalGridClient::getLocalGridTypes(){
	GetLocalGridTypesRequest request;
	assembleRequestHeader<GetLocalGridTypesRequest>(&request);
	return call<GetLocalGridTypesRequest, GetLocalGridTypesResponse>(request, &LocalGridService::Stub::GetLocalGridTypes);
}

GetLocalGridsResponse LocalGridClient::getLocalGrids(std::vector<LocalGridRequest> localGridRequests){
	GetLocalGridsRequest request;
	assembleRequestHeader<GetLocalGridsRequest>(&request);
	for (LocalGridRequest lgr : localGridRequests) {
    	request.add_local_grid_requests()->CopyFrom(lgr);
  	}
	return call<GetLocalGridsRequest, GetLocalGridsResponse>(request, &LocalGridService::Stub::GetLocalGrids);
}