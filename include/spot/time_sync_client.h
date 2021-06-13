#ifndef TIME_SYNC_CLIENT_H
#define TIME_SYNC_CLIENT_H

#include <spot/base_client.h>
#include "bosdyn/api/time_sync_service.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using grpc::ClientAsyncResponseReader;
using grpc::CompletionQueue;
using bosdyn::api::TimeSyncUpdateRequest;
using bosdyn::api::TimeSyncUpdateResponse;
using bosdyn::api::TimeSyncService;
using bosdyn::api::TimeSyncEstimate;
using bosdyn::api::TimeSyncRoundTrip;
using bosdyn::api::TimeSyncState;
using google::protobuf::Duration;
using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil;

class TimeSyncClient : public BaseClient<TimeSyncService> {
    public: 
    TimeSyncClient(const std::string &root, const std::string &server);

    //TimeSyncUpdateResponse EstablishTimeSync(const int& numRounds);

	TimeSyncUpdateResponse getTimeSyncUpdate(const TimeSyncRoundTrip &previousRoundTrip, const std::string &clockIdentifier);
	TimeSyncUpdateResponse getTimeSyncUpdateAsync(const TimeSyncRoundTrip &previousRoundTrip, const std::string &clockIdentifier);
};

#endif
