#include <spot/clients/world_objects.h>

namespace ClientLayer {
    
    WorldObjectsClient::WorldObjectsClient(const std::string &authority, const std::string &token) : BaseClient(WORLD_OBJECTS_CLIENT_NAME, authority, token) {}

    ListWorldObjectResponse WorldObjectsClient::listWorldObjects() {
        ListWorldObjectRequest request;
        assembleRequestHeader<ListWorldObjectRequest>(&request);
        return call<ListWorldObjectRequest, ListWorldObjectResponse>(request, &WorldObjectService::Stub::ListWorldObjects);
    }

    ListWorldObjectResponse WorldObjectsClient::listWorldObjects(Timestamp timeStampFilter){
        ListWorldObjectRequest request;
        assembleRequestHeader<ListWorldObjectRequest>(&request);
        request.mutable_timestamp_filter()->CopyFrom(timeStampFilter);
        return call<ListWorldObjectRequest, ListWorldObjectResponse>(request, &WorldObjectService::Stub::ListWorldObjects);
    }

    ListWorldObjectResponse WorldObjectsClient::listWorldObjects(std::vector<WorldObjectType> objectTypes, Timestamp timeStampFilter){
        ListWorldObjectRequest request;
        assembleRequestHeader<ListWorldObjectRequest>(&request);
        request.mutable_timestamp_filter()->CopyFrom(timeStampFilter);
        for(WorldObjectType wot : objectTypes) {
            request.add_object_type(wot);
        }
        return call<ListWorldObjectRequest, ListWorldObjectResponse>(request, &WorldObjectService::Stub::ListWorldObjects);
    }

    MutateWorldObjectResponse WorldObjectsClient::mutateWorldObjects(MutateWorldObjectRequest_Mutation mutation){
        MutateWorldObjectRequest request;
        assembleRequestHeader<MutateWorldObjectRequest>(&request);
        request.mutable_mutation()->CopyFrom(mutation);
        return call<MutateWorldObjectRequest, MutateWorldObjectResponse>(request, &WorldObjectService::Stub::MutateWorldObjects);
    }

};