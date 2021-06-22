#include <spot/clients/estop.h>

const std::string ESTOP_CLIENT_NAME = "estop";

// TODO: IMPLEMENT NEW ENDPOINT SYSTEM AND FINISH THREad

Endpoint::Endpoint(std::shared_ptr<EstopClient> client, const std::string &name, const std::string &role, const std::string &targetConfigId, 
		const std::string &uniqueId, int64_t estopTimeout, int64_t estopCutPowerTimeout) :
		_name(name),
		_client(client),
		_role(role),
		_estopTimeout(estopTimeout),
		_estopCutPowerTimeout(estopCutPowerTimeout),
		_configId(configId),
		_uniqueId(uniqueId) {
	// send the first check in request to retrieve the challenge
	EstopCheckInResponse reply;
	try {
		reply = _client->checkIn(EstopStopLevel::ESTOP_LEVEL_CUT, toProto(), _challenge, ~_challenge, suppressIncorrect);
	} catch (Error &error) {
		throw;
	}
	// set challenge
	setChallenge(reply.challenge());
}

void Endpoint::cut() {
	checkIn(EstopStopLevel::ESTOP_LEVEL_CUT);
}

void Endpoint::settleThenCut() {
	checkIn(EstopStopLevel::ESTOP_LEVEL_SETTLE_THEN_CUT);
}

void Endpoint::allow() {
	checkIn(EstopStopLevel::ESTOP_LEVEL_NONE);
}

void Endpoint::checkIn(EstopStopLevel level, bool suppressIncorrect) {
	EstopCheckInResponse reply;
	try {
		reply = _client->checkIn(level, toProto(), _challenge, ~_challenge, suppressIncorrect);
	} catch (Error &error) {
		throw;
	}
	// handle errors
	switch (reply.status()) {
		case 0:
			break;
		case 1:
			// set new challenge
			setChallenge(reply.challenge());
			break;
		case 2:
			break;
		case 3:
			break;
	}
}

void Endpoint::setChallenge(uint64_t newChallenge) {
	std::lock_guard<std::mutex> locker(_mu);
	_challenge = newChallenge;
}

EstopEndpoint Endpoint::toProto() {
	EstopEndpoint ret;
	ret.set_role(_role);
	ret.set_name(_name);
	ret.set_unique_id(_uniqueId);
	ret.mutable_timeout(TimeUtil::SecondsToDuration(_estopTimeout));
	ret.mutable_cut_power_timeout(TimeUtil::SecondsToDuration(_estopCutPowerTimeout));
	return ret;
}
 
EstopClient::EstopClient(const std::string &authority, const std::string &token) : BaseClient(ESTOP_CLIENT_NAME, authority, token) {}

RegisterEstopEndpointResponse EstopClient::registerEndpoint(const std::string &targetConfigId, EstopEndpoint &endpoint) {
	RegisterEstopEndpointRequest request;
    assembleRequestHeader<RegisterEstopEndpointRequest>(&request);
	request.set_target_config_id(targetConfigId);
	request.mutable_new_endpoint()->CopyFrom(endpoint);
    return call<RegisterEstopEndpointRequest, RegisterEstopEndpointResponse>(request, &EstopService::Stub::RegisterEstopEndpoint);
}

RegisterEstopEndpointResponse EstopClient::registerEndpointAsync(const std::string &targetConfigId, EstopEndpoint &endpoint) {
	RegisterEstopEndpointRequest request;
    assembleRequestHeader<RegisterEstopEndpointRequest>(&request);
	request.set_target_config_id(targetConfigId);
	request.mutable_new_endpoint()->CopyFrom(endpoint);
    return callAsync<RegisterEstopEndpointRequest, RegisterEstopEndpointResponse>(request, &EstopService::Stub::AsyncRegisterEstopEndpoint);
}

RegisterEstopEndpointResponse replaceEndpoint(const std::string &targetConfigId, const std::string &uniqueId, EstopEndpoint &endpoint) {
	RegisterEstopEndpointRequest request;
	assembleRequestHeader<RegisterEstopEndpointRequest>(&request);
	request.set_target_config_id(targetConfigId);
	request.mutable_new_endpoint()->CopyFrom(endpoint);
	request.mutable_target_endpoint()->unique_id(uniqueId);
	return call<RegisterEstopEndpointRequest, RegisterEstopEndpointResponse>(request, &EstopService::Stub::RegisterEstopEndpoint);
}

RegisterEstopEndpointResponse replaceEndpointAsync(const std::string &targetConfigId, const std::string &uniqueId, EstopEndpoint &endpoint) {
	RegisterEstopEndpointRequest request;
	assembleRequestHeader<RegisterEstopEndpointRequest>(&request);
	request.set_target_config_id(targetConfigId);
	request.mutable_new_endpoint()->CopyFrom(endpoint);
	request.mutable_target_endpoint()->unique_id(uniqueId);
	return callAsync<RegisterEstopEndpointRequest, RegisterEstopEndpointResponse>(request, &EstopService::Stub::RegisterEstopEndpoint);
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

EstopThread::EstopThread(std::shared_ptr<EstopClient> clientPtr, Endpoint &endpoint) :
		_client(clientPtr),
		_endpoint(endpoint) {}

EstopThread::~EstopThread() {
	endEstop();
}

EstopThread::beginEstop() {
	_keepRunning = true;

	// endpoint passed in already has valid challenge / response pair
	_thread = std::shared_ptr<std::thread>(new std::thread(EstopThread::periodicCheckIn, this));
}

EstopThread::endEstop() {
	_keepRunning = false;
	_thread->join(); // should wait for thread to stop
}

void EstopThread::periodicCheckIn() {
	while (_keepRunning) {
		// will update challenge / response within endpoint obj
		_endpoint.checkIn(EstopStopLevel::ESTOP_LEVEL_NONE, false);

		// todo: timeouts, etc.
	}
	return;
}