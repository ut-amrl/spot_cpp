/*
  timesync.h: includes client and interface for communication with the timesync service 
*/

#ifndef TIMESYNC_H
#define TIMESYNC_H

#include <spot/base.h>
#include "bosdyn/api/time_sync_service.grpc.pb.h"

using bosdyn::api::TimeSyncUpdateRequest;
using bosdyn::api::TimeSyncUpdateResponse;
using bosdyn::api::TimeSyncService;
using bosdyn::api::TimeSyncEstimate;
using bosdyn::api::TimeSyncRoundTrip;
using bosdyn::api::TimeSyncState;

const extern std::string TIMESYNC_CLIENT_NAME;

class TimeSyncClient : public BaseClient<TimeSyncService> {
public: 
  TimeSyncClient(const std::string &authority, const std::string &token);

  TimeSyncUpdateResponse getTimeSyncUpdate();
  TimeSyncUpdateResponse getTimeSyncUpdate(const TimeSyncRoundTrip &previousRoundTrip);
  TimeSyncUpdateResponse getTimeSyncUpdate(const TimeSyncRoundTrip &previousRoundTrip, const std::string &clockIdentifier);
  TimeSyncUpdateResponse getTimeSyncUpdateAsync(const TimeSyncRoundTrip &previousRoundTrip, const std::string &clockIdentifier);
};

class TimeSyncKeepAlive {
public:
  TimeSyncKeepAlive(std::shared_ptr<TimeSyncClient> clientPtr, const std::string &clockIdentifier);
  ~TimeSyncKeepAlive();

private:
  void periodicCheckIn();
  void checkIn();

private:
  std::shared_ptr<TimeSyncClient> _clientPtr;
};

#endif
