#include <spot/clients/timesync.h>

const std::string TIMESYNC_CLIENT_NAME = "time-sync";

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

TimeSyncThread::TimeSyncThread(std::shared_ptr<TimeSyncClient> clientPtr, const std::string &clockIdentifier) :
		_client(clientPtr) {}
		
TimeSyncThread::~TimeSyncThread() {
	endTimeSync();
}

/* createTrip(): helper method to create a new round trup from a previous response
   Input: Old TimeSyncUpdateReponse
   Output: TimeSyncRoundTrip
   Side effects: -
*/
TimeSyncRoundTrip createTrip(TimeSyncUpdateResponse &reply) {
	TimeSyncRoundTrip trip;
	trip.mutable_client_rx()->CopyFrom(TimeUtil::GetCurrentTime());
	trip.mutable_client_tx()->CopyFrom(reply.header().request_header().request_timestamp());
	trip.mutable_server_tx()->CopyFrom(reply.header().response_timestamp());
	trip.mutable_server_rx()->CopyFrom(reply.header().request_received_timestamp());
}

void TimeSyncThread::beginTimeSync() {
	// initial rpc
	TimeSyncUpdateReponse reply;
	try {
		reply = _client->getTimeSyncUpdate();
	} catch (Error &error) {
		throw;
	}

	// set clock id
	_clockIdentifier = reply.clock_identifier();

	// send rpcs until synchronized
	while (reply.state().status() == 2 || reply.state().status() == 3) {
		// send new rpc and set clockskew
		reply = _client->getTimeSyncUpdate(createTrip(reply), _timeSyncClockId);
		_clockSkew = TimeUtil::DurationToSeconds(reply.state().best_estimate().clock_skew());
		std::this_thread::sleep_for(std::chrono::seconds(DEFAULT_TIME_SYNC_NOT_READY_INTERVAL_SECONDS));
	}

	// clock skew available, thread is ready to be kicked off
	_keepRunning = true;
	_thread = std::shared_ptr<std::thread>(new std::thread(TimeSyncThread::periodicCheckIn, this));
}

void TimeSyncThread::endTimeSync() {
	// turn off keep running
	setKeepRunning(false);

	// join thread
	_thread->join();
}

void TimeSyncThread::periodicCheckIn(TimeSyncRoundTrip &init) {
	TimeSyncUpdateResponse reply = init;
	while (_keepRunning) {
		// send new rpc and set clockskew
		reply = _timeSyncClientPtr->getTimeSyncUpdate(createTrip(reply), _timeSyncClockId);
		_clockSkew = TimeUtil::DurationToSeconds(reply.state().best_estimate().clock_skew());
		std::this_thread::sleep_for(std::chrono::seconds(DEFAULT_TIME_SYNC_INTERVAL_SECS));
	}
	return;
}

