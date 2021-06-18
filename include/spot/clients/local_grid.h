#ifndef LOCAL_GRID_H
#define LOCAL_GRID_H

#include "bosdyn/api/local_grid_service.grpc.pb.h"
#include <spot/clients/base.h>

using bosdyn::api::LocalGridService;
using bosdyn::api::GetLocalGridTypesResponse;
using bosdyn::api::GetLocalGridTypesRequest;
using bosdyn::api::GetLocalGridsResponse;
using bosdyn::api::GetLocalGridsRequest;
using bosdyn::api::LocalGridRequest;

const extern std::string LOCAL_GRID_CLIENT_NAME;

class LocalGridClient : public BaseClient<LocalGridService> {
public:
  LocalGridClient(const std::string &authority, const std::string &token);

  GetLocalGridTypesResponse getLocalGridTypes();
  GetLocalGridsResponse getLocalGrids(std::vector<LocalGridRequest> localGridRequests);
};

#endif