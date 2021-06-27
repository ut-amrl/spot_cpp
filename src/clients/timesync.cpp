#include <spot/clients/timesync.h>

namespace ClientLayer {

	TimeSyncClient::TimeSyncClient(const std::string &authority, const std::string &token) : BaseClient(TIMESYNC_CLIENT_NAME, authority, token) {}

	TimeSyncUpdateResponse TimeSyncClient::getTimeSyncUpdate() {
		TimeSyncUpdateRequest request;
		assembleRequestHeader<TimeSyncUpdateRequest>(&request);
		return call<TimeSyncUpdateRequest, TimeSyncUpdateResponse>(request, &TimeSyncService::Stub::TimeSyncUpdate);
	}
			
	TimeSyncUpdateResponse TimeSyncClient::getTimeSyncUpdate(const TimeSyncRoundTrip &previousRoundTrip, const std::string &clockIdentifier) {
		TimeSyncUpdateRequest request;
		assembleRequestHeader<TimeSyncUpdateRequest>(&request);
		request.set_clock_identifier(clockIdentifier);
		request.mutable_previous_round_trip()->CopyFrom(previousRoundTrip);
		return call<TimeSyncUpdateRequest, TimeSyncUpdateResponse>(request, &TimeSyncService::Stub::TimeSyncUpdate);
	}

	TimeSyncUpdateResponse TimeSyncClient::getTimeSyncUpdateAsync(const TimeSyncRoundTrip &previousRoundTrip, const std::string &clockIdentifier) {
		TimeSyncUpdateRequest request;
		assembleRequestHeader<TimeSyncUpdateRequest>(&request);
		request.set_clock_identifier(clockIdentifier);
		request.mutable_previous_round_trip()->CopyFrom(previousRoundTrip);
		return callAsync<TimeSyncUpdateRequest, TimeSyncUpdateResponse>(request, &TimeSyncService::Stub::AsyncTimeSyncUpdate);
	}

	int TimeSyncEndpoint::DEFAULT_MAX_SAMPLES = 25;

	TimeSyncEndpoint::TimeSyncEndpoint(std::shared_ptr<TimeSyncClient> client) :
			_client(client) {} 

	bool TimeSyncEndpoint::hasEstablishedTimeSync() {
		std::lock_guard<std::mutex> locker(_mu);
		// check if response exists and the status is ok
		return _lockedPreviousResponse && _lockedPreviousResponse->state().status() == 1;
	}

	google::protobuf::Duration TimeSyncEndpoint::roundTripTime() {
		std::lock_guard<std::mutex> locker(_mu);

		if (!_lockedPreviousResponse) {
			std::cout << "timesync not established" << std::endl; // exceptions later
		}

		// now we can return the round trip time
		return _lockedPreviousResponse->state().best_estimate().round_trip_time();
	}

	google::protobuf::Duration TimeSyncEndpoint::clockSkew() {
		std::lock_guard<std::mutex> locker(_mu);

		// if no response
		if (!_lockedPreviousResponse) {
			std::cout << "timesync not established" << std::endl;
		}

		return _lockedPreviousResponse->state().best_estimate().clock_skew();
	}

	bool TimeSyncEndpoint::establishTimeSync(int maxSamples, bool breakOnSuccess) {
		while (maxSamples-- != 0) {
			if (breakOnSuccess && hasEstablishedTimeSync()) {
				return true;
			}
			getNewEstimate();
		}

		return hasEstablishedTimeSync();
	}

	bool TimeSyncEndpoint::getNewEstimate() {
		TimeSyncUpdateResponse reply = _getUpdate();
		google::protobuf::Timestamp now = google::protobuf::util::TimeUtil::GetCurrentTime();

		// record round trip info
		TimeSyncRoundTrip trip;
		trip.mutable_client_tx()->CopyFrom(reply.header().request_header().request_timestamp());
		trip.mutable_server_rx()->CopyFrom(reply.header().request_received_timestamp());
		trip.mutable_server_tx()->CopyFrom(reply.header().response_timestamp());
		trip.mutable_client_rx()->CopyFrom(now);

		std::lock_guard<std::mutex> locker(_mu);
		_lockedPreviousRoundTrip.reset(std::addressof(trip));
		_lockedPreviousResponse.reset(std::addressof(reply));
		_lockedClockIdentifier = reply.clock_identifier();
	}

	google::protobuf::Timestamp TimeSyncEndpoint::robotTimestampFromLocalTimestamp(google::protobuf::Timestamp localTimestamp) {
		// convert local timestamp to time_t
		time_t localTime = google::protobuf::util::TimeUtil::TimestampToTimeT(localTimestamp);

		// convert skew to seconds
		int64_t secs = google::protobuf::util::TimeUtil::DurationToSeconds(clockSkew());

		// add the two (assumes time_t in seconds)
		time_t robotTime = localTime + seconds;

		// return timestamp of robottime
		return google::protobuf::util::TimeUtil::TimeTToTimestamp(robotTime);
	}

	const TimeSyncRoundTrip TimeSyncEndpoint::getPreviousRoundTrip() {
		std::lock_guard<std::mutex> locker(_mu);
		TimeSyncRoundTrip ret = *_lockedPreviousRoundTrip;
		return ret;
	}

	const TimeSyncUpdateResponse TimeSyncEndpoint::getPreviousResponse() {
		std::lock_guard<std::mutex> locker(_mu);
		TimeSyncUpdateResponse ret = *_lockedPreviousResponse;
		return ret;
	}

	const std::string TimeSyncEndpoint::getClockIdentifier() {
		std::lock_guard<std::mutex> locker(_mu);
		std::string ret = _lockedClockIdentifier;
		return ret;
	}

	const TimeSyncUpdateResponse TimeSyncEndpoint::_getUpdate() {
		TimeSyncRoundTrip trip;
		std::string clockid;

		std::lock_guard<std::mutex> locker(_mu);
		if (!_lockedClockIdentifier.empty()) {
			trip = *_lockedPreviousRoundTrip;
			clockid = _lockedClockIdentifier;
		}

		// send rpc
		_client->getTimeSyncUpdate(trip, clockid);
	}

	TimeSyncThread::TimeSyncThread(std::shared_ptr<TimeSyncClient> clientPtr, const std::string &clockIdentifier, int64_t initClockSkew, TimeSyncUpdateResponse response) :
			_client(clientPtr),
			_clockIdentifier(clockIdentifier),
			_clockSkew(initClockSkew),
			_response(response) {}
			
	TimeSyncThread::~TimeSyncThread() {
		endTimeSync();
	}

	int TimeSyncThread::DEFAULT_TIME_SYNC_INTERVAL_SECONDS = 60;

	void TimeSyncThread::beginTimeSync() {
		// clock skew available, thread is ready to be kicked off
		_keepRunning = true;
		_thread = std::shared_ptr<std::thread>(new std::thread(&TimeSyncThread::periodicCheckIn, this));
	}

	void TimeSyncThread::endTimeSync() {

		// turn off keep running
		setKeepRunning(false);

		// join thread
		if (_thread->joinable()){
			_thread->join();
		}
	}

	void TimeSyncThread::periodicCheckIn() {
		TimeSyncUpdateResponse reply = _response;
		while (_keepRunning) {
			// send new rpc and set clockskew
			reply = _client->getTimeSyncUpdate(createTrip(reply), _clockIdentifier);
			_clockSkew = TimeUtil::DurationToSeconds(reply.state().best_estimate().clock_skew());
			std::this_thread::sleep_for(std::chrono::seconds(DEFAULT_TIME_SYNC_INTERVAL_SECONDS));
		}
		return;
	}

	TimeSyncRoundTrip createTrip(TimeSyncUpdateResponse &reply) {
		TimeSyncRoundTrip trip;
		trip.mutable_client_rx()->CopyFrom(TimeUtil::GetCurrentTime());
		trip.mutable_client_tx()->CopyFrom(reply.header().request_header().request_timestamp());
		trip.mutable_server_tx()->CopyFrom(reply.header().response_timestamp());
		trip.mutable_server_rx()->CopyFrom(reply.header().request_received_timestamp());
		return trip;
	}

};