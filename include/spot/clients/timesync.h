/*
  timesync.h: includes client and interface for communication with the timesync service 
*/

#ifndef TIMESYNC_H
#define TIMESYNC_H

#include <spot/clients/base.h>
#include "bosdyn/api/time_sync_service.grpc.pb.h"

using bosdyn::api::TimeSyncUpdateRequest;
using bosdyn::api::TimeSyncUpdateResponse;
using bosdyn::api::TimeSyncService;
using bosdyn::api::TimeSyncEstimate;
using bosdyn::api::TimeSyncRoundTrip;
using bosdyn::api::TimeSyncState;

const extern std::string TIMESYNC_CLIENT_NAME;
static int 

class TimeSyncClient : public BaseClient<TimeSyncService> {
public: 
  TimeSyncClient(const std::string &authority, const std::string &token);

  TimeSyncUpdateResponse getTimeSyncUpdate(); // used to get clock identifier initially
  TimeSyncUpdateResponse getTimeSyncUpdate(const TimeSyncRoundTrip &previousRoundTrip, const std::string &clockIdentifier);
  TimeSyncUpdateResponse getTimeSyncUpdate();
  TimeSyncUpdateResponse getTimeSyncUpdateAsync(const TimeSyncRoundTrip &previousRoundTrip, const std::string &clockIdentifier);
};

class TimeSyncThread {
public:
  TimeSyncThread(std::shared_ptr<TimeSyncClient> clientPtr, const std::string &clockIdentifier, int64_t initClockSkew);
  ~TimeSyncThread();
  
  /* beginTimeSync(): does intitial RPC to timesync service and kicks off thread for constant check-ins
     Input: -
     Output: -
     Side effects: creates thread
  */
  void beginTimeSync();

  /* endTimeSync(): kills the thread running the timesync checkins
     Input: -
     Output: -
     Side effects: kill thread
  */
 void endTimeSync();

  /* Mutators */
  void setKeepRunning(bool keepRunning) { _keepRunning = keepRunning; }

  /* Accessors */
  const std::shared_ptr<std::thread> getThread() const { return _thread; }
  const std::string getClockIdentifier() const { return _clockIdentifier; }
  const std::string getClockSkew() const { return _clockSkew; }

private:
  /* periodicCheckIn(): function that thread runs, loop depends on boolean set by main thread
     Input: -
     Output: -
     Side effects: issues RPCs to timesync service
  */
  void periodicCheckIn();

private:
  std::shared_ptr<TimeSyncClient> _client;
  std::shared_ptr<std::thread> _thread;
  
  std::string _clockIdentifier;
  int64_t _clockSkew;
  bool _keepRunning;
};

#endif
