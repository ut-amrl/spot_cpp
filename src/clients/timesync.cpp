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

TimeSyncThread::TimeSyncThread(std::shared_ptr<TimeSyncClient> clientPtr, const std::string &clockIdentifier, int64_t initClockSkew) :
		_client(clientPtr),
		_clockIdentifier(clockIdentifier),
		_clockSkew(initClockSkew) {}
		
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

	std::cout << "setKeepRunning()" << std::endl;
	// turn off keep running
	setKeepRunning(false);

	std::cout << "_thread->join()" << std::endl;
	// join thread
	// _thread->join();
}

void TimeSyncThread::periodicCheckIn() {
	TimeSyncUpdateResponse reply;
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