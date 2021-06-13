#include <spot/time_sync_client.h>

TimeSyncClient::TimeSyncClient(const std::string &root, const std::string &server) {
	_stub = initializeNoAuthToken(server, root, "timesync.spot.robot");
	_clientName = "timesync";
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
