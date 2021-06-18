#include <spot/clients/estop.h>

const std::string ESTOP_CLIENT_NAME = "estop";

EstopClient::EstopClient(const std::string &authority, const std::string &token) : BaseClient(ESTOP_CLIENT_NAME, authority, token) {}

 
RegisterEstopEndpointResponse EstopClient::registerEndpoint(const std::string &targetConfigId, EstopEndpoint &targetEndpoint, EstopEndpoint &endpoint) {
	RegisterEstopEndpointRequest request;
    assembleRequestHeader<RegisterEstopEndpointRequest>(&request);
	request.set_target_config_id(targetConfigId);
	request.mutable_new_endpoint()->CopyFrom(endpoint);
	request.mutable_target_endpoint()->CopyFrom(targetEndpoint);
    return call<RegisterEstopEndpointRequest, RegisterEstopEndpointResponse>(request, &EstopService::Stub::RegisterEstopEndpoint);
}

RegisterEstopEndpointResponse EstopClient::registerEndpointAsync(const std::string &targetConfigId, EstopEndpoint &endpoint) {
	RegisterEstopEndpointRequest request;
    assembleRequestHeader<RegisterEstopEndpointRequest>(&request);
	request.set_target_config_id(targetConfigId);
	request.mutable_new_endpoint()->CopyFrom(endpoint);
    return callAsync<RegisterEstopEndpointRequest, RegisterEstopEndpointResponse>(request, &EstopService::Stub::AsyncRegisterEstopEndpoint);
}

DeregisterEstopEndpointResponse EstopClient::deregister(const std::string &targetConfigId, EstopEndpoint &endpoint) {
	DeregisterEstopEndpointRequest request;
    assembleRequestHeader<DeregisterEstopEndpointRequest>(&request);
	request.mutable_target_endpoint()->CopyFrom(endpoint);
	request.set_target_config_id(targetConfigId);
    return call<DeregisterEstopEndpointRequest, DeregisterEstopEndpointResponse>(request, &EstopService::Stub::DeregisterEstopEndpoint);
}

DeregisterEstopEndpointResponse EstopClient::deregisterAsync(const std::string &targetConfigId, EstopEndpoint &endpoint) {
	DeregisterEstopEndpointRequest request;
    assembleRequestHeader<DeregisterEstopEndpointRequest>(&request);
	request.mutable_target_endpoint()->CopyFrom(endpoint);
	request.set_target_config_id(targetConfigId);
    return callAsync<DeregisterEstopEndpointRequest, DeregisterEstopEndpointResponse>(request, &EstopService::Stub::AsyncDeregisterEstopEndpoint);
}

GetEstopConfigResponse EstopClient::getConfig(const std::string &targetConfigId) {
	GetEstopConfigRequest request;
    assembleRequestHeader<GetEstopConfigRequest>(&request);
	request.set_target_config_id(targetConfigId);
    return call<GetEstopConfigRequest, GetEstopConfigResponse>(request, &EstopService::Stub::GetEstopConfig);
}

GetEstopConfigResponse EstopClient::getConfigAsync(const std::string &targetConfigId) {
	GetEstopConfigRequest request;
    assembleRequestHeader<GetEstopConfigRequest>(&request);
	request.set_target_config_id(targetConfigId);
    return callAsync<GetEstopConfigRequest, GetEstopConfigResponse>(request, &EstopService::Stub::AsyncGetEstopConfig);
}

SetEstopConfigResponse EstopClient::setConfig(EstopConfig &config){
	SetEstopConfigRequest request;
    assembleRequestHeader<SetEstopConfigRequest>(&request);
	request.mutable_config()->CopyFrom(config);
    return call<SetEstopConfigRequest, SetEstopConfigResponse>(request, &EstopService::Stub::SetEstopConfig);
}

SetEstopConfigResponse EstopClient::setConfig(EstopConfig &config, std::string targetConfigId) {
	SetEstopConfigRequest request;
    assembleRequestHeader<SetEstopConfigRequest>(&request);
	request.mutable_config()->CopyFrom(config);
	request.set_target_config_id(targetConfigId);
    return call<SetEstopConfigRequest, SetEstopConfigResponse>(request, &EstopService::Stub::SetEstopConfig);
}

SetEstopConfigResponse EstopClient::setConfigAsync(EstopConfig &config, std::string targetConfigId) {
	SetEstopConfigRequest request;
    assembleRequestHeader<SetEstopConfigRequest>(&request);
	request.mutable_config()->CopyFrom(config);
	request.set_target_config_id(targetConfigId);
    return callAsync<SetEstopConfigRequest, SetEstopConfigResponse>(request, &EstopService::Stub::AsyncSetEstopConfig);
}

GetEstopSystemStatusResponse EstopClient::getStatus() {
	GetEstopSystemStatusRequest request;
    assembleRequestHeader<GetEstopSystemStatusRequest>(&request);
    return call<GetEstopSystemStatusRequest, GetEstopSystemStatusResponse>(request, &EstopService::Stub::GetEstopSystemStatus);
}

GetEstopSystemStatusResponse EstopClient::getStatusAsync() {
	GetEstopSystemStatusRequest request;
    assembleRequestHeader<GetEstopSystemStatusRequest>(&request);
    return callAsync<GetEstopSystemStatusRequest, GetEstopSystemStatusResponse>(request, &EstopService::Stub::AsyncGetEstopSystemStatus);
}

EstopCheckInResponse EstopClient::checkIn(EstopStopLevel &stopLevel, EstopEndpoint &endpoint, uint64_t challenge, uint64_t response, bool suppress_incorrect) {
	// TODO: implement logic for suppress_incorrect
	EstopCheckInRequest request;
    assembleRequestHeader<EstopCheckInRequest>(&request);
	request.set_challenge(challenge);
	request.set_response(response);
	request.mutable_endpoint()->CopyFrom(endpoint);
	request.set_stop_level(stopLevel);
    return call<EstopCheckInRequest, EstopCheckInResponse>(request, &EstopService::Stub::EstopCheckIn);
}

EstopCheckInResponse EstopClient::checkInAsync(EstopStopLevel &stopLevel, EstopEndpoint &endpoint, uint64_t challenge, uint64_t response, bool suppress_incorrect) {
	// TODO: implement logic for suppress_incorrect
	EstopCheckInRequest request;
    assembleRequestHeader<EstopCheckInRequest>(&request);
	request.set_challenge(challenge);
	request.set_response(response);
	request.mutable_endpoint()->CopyFrom(endpoint);
	request.set_stop_level(stopLevel);
    return callAsync<EstopCheckInRequest, EstopCheckInResponse>(request, &EstopService::Stub::AsyncEstopCheckIn);
}

EstopKeepAlive::EstopKeepAlive(std::shared_ptr<EstopClient> clientPtr, EstopEndpoint endpoint, EstopStopLevel stopLevel, int rpcTimeoutSeconds, int rpcIntervalSeconds, uint64_t challenge) :
		_clientPtr(clientPtr),
		_endpoint(endpoint),
		_stopLevel(stopLevel),
		_rpcTimeoutSeconds(rpcTimeoutSeconds),
		_rpcIntervalSeconds(rpcIntervalSeconds),
		_challenge(challenge),
		_keepRunning(true) { // create thread
		_thread = std::thread(&EstopKeepAlive::periodicCheckIn, this);

}

EstopKeepAlive::~EstopKeepAlive() {
	_thread.std::thread::~thread();
}

void EstopKeepAlive::periodicCheckIn() {
	// do first check in and set new challenge
	EstopCheckInResponse reply = _clientPtr->checkIn(_stopLevel, _endpoint, _challenge, ~_challenge, false);
	_challenge = reply.challenge();
	while(true) {
		checkIn();
		sleep(1);
	}
}

void EstopKeepAlive::checkIn() {
	EstopCheckInResponse reply = _clientPtr->checkIn(_stopLevel, _endpoint, _challenge, ~_challenge, false);
	_challenge = reply.challenge(); // update challenge
}
