#ifndef WORLD_OBJECTS_H
#define WORLD_OBJECTS_H

#include "bosdyn/api/world_object_service.grpc.pb.h"
#include <spot/clients/base.h>

using bosdyn::api::WorldObjectService;
using bosdyn::api::WorldObjectType;
using bosdyn::api::ListWorldObjectResponse;
using bosdyn::api::ListWorldObjectRequest;
using bosdyn::api::MutateWorldObjectResponse;
using bosdyn::api::MutateWorldObjectRequest;
using bosdyn::api::MutateWorldObjectRequest_Mutation;
using google::protobuf::Timestamp;

namespace ClientLayer {

    class WorldObjectsClient : public BaseClient<WorldObjectService> {
    public:
    WorldObjectsClient(const std::string &authority, const std::string &token);

    ListWorldObjectResponse listWorldObjects(Timestamp timeStampFilter);
    ListWorldObjectResponse listWorldObjects(std::vector<WorldObjectType> objectTypes, Timestamp timeStampFilter);

    MutateWorldObjectResponse mutateWorldObjects(MutateWorldObjectRequest_Mutation mutation);
    };

};

#endif