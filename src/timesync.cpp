#include <spot/timesync.h>

const std::string TIMESYNC_CLIENT_NAME = "time-sync";

TimeSyncClient::TimeSyncClient(const std::string &authority, const std::string &token) : BaseClient(TIMESYNC_CLIENT_NAME, authority, token) {}

TimeSyncUpdateResponse TimeSyncClient::getTimeSyncUpdate() {
	TimeSyncUpdateRequest request;
	assembleRequestHeader<TimeSyncUpdateRequest>(&request);
	return call<TimeSyncUpdateRequest, TimeSyncUpdateResponse>(request, &TimeSyncService::Stub::TimeSyncUpdate);
}

TimeSyncUpdateResponse TimeSyncClient::getTimeSyncUpdate(const TimeSyncRoundTrip &previousRoundTrip) {
	TimeSyncUpdateRequest request;
	assembleRequestHeader<TimeSyncUpdateRequest>(&request);
	request.mutable_previous_round_trip()->CopyFrom(previousRoundTrip);
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

TimeSyncKeepAlive::TimeSyncKeepAlive(std::shared_ptr<TimeSyncClient> clientPtr, const std::string &clockIdentifier) :
		_clientPtr(clientPtr) {
}

TimeSyncKeepAlive::~TimeSyncKeepAlive() {

}

void TimeSyncKeepAlive::periodicCheckIn() {

}

void TimeSyncKeepAlive::checkIn() {

}
