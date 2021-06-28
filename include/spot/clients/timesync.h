/*
  timesync.h: includes client and interface for communication with the timesync service 
*/

#ifndef TIMESYNC_H
#define TIMESYNC_H

#include <spot/clients/base.h>
#include "bosdyn/api/time_sync_service.grpc.pb.h"

#include "time.h"

using bosdyn::api::TimeSyncUpdateRequest;
using bosdyn::api::TimeSyncUpdateResponse;
using bosdyn::api::TimeSyncService;
using bosdyn::api::TimeSyncEstimate;
using bosdyn::api::TimeSyncRoundTrip;
using bosdyn::api::TimeSyncState;

// TODO: provide functionality for estimation of robot clock time for movement methods (sdk)
// timestamps need to be specified relative to the robot's system clock, so we need to find a way to convert client time into robot time

namespace ClientLayer {

  /*
    class TimeSyncClient: A client for establishing time-sync with server/robot, essentially a wrapper around the TimeSyncEndpoint object
  */
  class TimeSyncClient : public BaseClient<TimeSyncService> {
  public: 
    TimeSyncClient(const std::string &authority, const std::string &token);

    TimeSyncUpdateResponse getTimeSyncUpdate(); // used to get clock identifier initially
    TimeSyncUpdateResponse getTimeSyncUpdate(const TimeSyncRoundTrip &previousRoundTrip, const std::string &clockIdentifier);
    TimeSyncUpdateResponse getTimeSyncUpdateAsync();
    TimeSyncUpdateResponse getTimeSyncUpdateAsync(const TimeSyncRoundTrip &previousRoundTrip, const std::string &clockIdentifier);
  };

  /*
    class TimeSyncEndpoint: A wrapper that uses a TimeSyncClient object to establish and maintain timesync with a robot.
    This class manages internal state, including a clock identifier and previous best time sync
    estimates. This class automatically builds requests passed to the TimeSyncClient, so users
    don't have to worry about the details of establishing and maintaining timesync. 

    This object is thread-safe.
  */
  class TimeSyncEndpoint {
  public:
    static int DEFAULT_MAX_SAMPLES;

    TimeSyncEndpoint(std::shared_ptr<TimeSyncClient> client);

    /*
      hasEstablishedTimeSync(): returns whether this endpoint has already established time-sync or not
      Input: -
      Output: true if established, false if not
      Side effects: -
    */
    bool hasEstablishedTimeSync();

    /*
      roundTripTime(): returns the previous round trip time as a google.protobuf.Duration
      Input: -
      Output: previous round trip time
      Side effects: -
    */
    google::protobuf::Duration roundTripTime();

    /*
      clockSkew(): returns the best estimate of the clock skew from the time-sync service as a google.protobuf.Duration
      Input: -
      Output: best clock skew estimate
    */
    google::protobuf::Duration clockSkew();

    /*
      establishTimesync(): establish timesync with service
      Input: maximum number of times to attempt to establish timesync, boolean that if true, stop performing time-sync after established
      Output: true if success, false if not
      Side effects: sets member variables
    */
    bool establishTimeSync(int maxSamples, bool breakOnSuccess);

    /*
      getNewEstimate(): perform update-cycle towards achieving time-synchronization
      Input: -
      Output: true if success, false if not
      Side effects: sets member variables
    */
    bool getNewEstimate();
    
    /*
      robotTimestampFromLocalTimestamp(): convert a local timestamp (e.g. from GetCurrentTime()) and converts into robot time
      Input: local timestamp
      Output: timestamp in robot time
      Side effects: -
    */
    google::protobuf::Timestamp robotTimestampFromLocalTimestamp(google::protobuf::Timestamp localTimestamp);
    
    /* Accessors using lock */
    const TimeSyncRoundTrip getPreviousRoundTrip();
    const TimeSyncUpdateResponse getPreviousResponse();
    const std::string getClockIdentifier();

  private:
    const TimeSyncUpdateResponse _getUpdate();

  private:
    std::shared_ptr<TimeSyncClient> _client;
    std::mutex _mu;

    // access using the lock, should be updated by copy not reference so they can be used outside the lock after being accessed with the lock
    std::unique_ptr<TimeSyncRoundTrip> _lockedPreviousRoundTrip;
    std::unique_ptr<TimeSyncUpdateResponse> _lockedPreviousResponse;
    std::string _lockedClockIdentifier;
  };

  /*
    class TimeSyncThread: Background thread for maintaining and achieving time-sync to the robot, essentially a wrapper around the TimeSyncEndpoint class
  */
  class TimeSyncThread {
  public:
    static int DEFAULT_TIME_SYNC_INTERVAL_SECONDS;
    static int DEFAULT_TIME_SYNC_NOT_AVAILABLE_SECONDS;

    TimeSyncThread(std::shared_ptr<TimeSyncClient> clientPtr);
    ~TimeSyncThread();
    
    /* start(): kicks off the thread
       Input: -
       Output: -
       Side effects: kicks off thread
    */
    void start();

    /* stop(): kills the thread running the timesync checkins
      Input: -
      Output: -
      Side effects: kill thread
    */
    void stop();

    /* Thread-safe mutators */
    void setKeepRunning(bool keepRunning);

    /* Thread-safe accessors */
    bool getKeepRunning();

    /* Accessors */
    const std::shared_ptr<std::thread> getThread() const { return _thread; }
    const std::shared_ptr<TimeSyncEndpoint> getEndpoint() const { return _endpoint; }

  private:
    /* _timeSyncThread(): function that thread runs, loop depends on boolean set by main thread
      Input: -
      Output: -
      Side effects: issues RPCs to timesync service
    */
    void _timeSyncThread();

  private:
    std::shared_ptr<TimeSyncClient> _client;
    std::shared_ptr<std::thread> _thread;
    std::shared_ptr<TimeSyncEndpoint> _endpoint;
    std::mutex _mu;

    bool _lockedKeepRunning;
  };

  /* createTrip(): helper method to create a new round trip from a previous response
    Input: Old TimeSyncUpdateReponse
    Output: TimeSyncRoundTrip
    Side effects: -
  */
  TimeSyncRoundTrip createTrip(TimeSyncUpdateResponse &reply);

};

#endif
