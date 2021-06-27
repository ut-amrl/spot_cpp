#include <spot/clients/estop.h>

namespace ClientLayer {

	EstopEndpoint::EstopEndpoint(std::shared_ptr<EstopClient> client, const std::string &name, const std::string &role, const std::string &configId, 
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
			EstopStopLevel stopLevel = EstopStopLevel::ESTOP_LEVEL_CUT;
			bosdyn::api::EstopEndpoint temp = toProto();
			reply = _client->checkIn(stopLevel, temp, _challenge, ~_challenge, true);
		} catch (Error &error) {
			throw;
		}
		// set challenge
		setChallenge(reply.challenge());
	}

	void EstopEndpoint::cut() {
		checkIn(EstopStopLevel::ESTOP_LEVEL_CUT, false);
	}

	void EstopEndpoint::settleThenCut() {
		checkIn(EstopStopLevel::ESTOP_LEVEL_SETTLE_THEN_CUT, false);
	}

	void EstopEndpoint::allow() {
		checkIn(EstopStopLevel::ESTOP_LEVEL_NONE, false);
	}

	// todo: timeouts
	void EstopEndpoint::checkIn(EstopStopLevel level, bool suppressIncorrect) {
		EstopCheckInResponse reply;
		try {
			bosdyn::api::EstopEndpoint temp = toProto();
			reply = _client->checkIn(level, temp, _challenge, ~_challenge, suppressIncorrect);
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

	void EstopEndpoint::setChallenge(uint64_t newChallenge) {
		std::lock_guard<std::mutex> locker(_mu);
		_challenge = newChallenge;
	}

	bosdyn::api::EstopEndpoint EstopEndpoint::toProto() {
		bosdyn::api::EstopEndpoint ret;
		ret.set_role(_role);
		ret.set_name(_name);
		ret.set_unique_id(_uniqueId);
		ret.mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(_estopTimeout));
		// ret.mutable_cut_power	_timeout()->CopyFrom(TimeUtil::SecondsToDuration(_estopCutPowerTimeout));
		return ret;
	}
	
	EstopClient::EstopClient(const std::string &authority, const std::string &token) : BaseClient(ESTOP_CLIENT_NAME, authority, token) {}

	RegisterEstopEndpointResponse EstopClient::registerEndpoint(const std::string &targetConfigId, bosdyn::api::EstopEndpoint &endpoint) {
		RegisterEstopEndpointRequest request;
		assembleRequestHeader<RegisterEstopEndpointRequest>(&request);
		request.set_target_config_id(targetConfigId);
		request.mutable_new_endpoint()->CopyFrom(endpoint);
		return call<RegisterEstopEndpointRequest, RegisterEstopEndpointResponse>(request, &EstopService::Stub::RegisterEstopEndpoint);
	}

	RegisterEstopEndpointResponse EstopClient::registerEndpointAsync(const std::string &targetConfigId, bosdyn::api::EstopEndpoint &endpoint) {
		RegisterEstopEndpointRequest request;
		assembleRequestHeader<RegisterEstopEndpointRequest>(&request);
		request.set_target_config_id(targetConfigId);
		request.mutable_new_endpoint()->CopyFrom(endpoint);
		return callAsync<RegisterEstopEndpointRequest, RegisterEstopEndpointResponse>(request, &EstopService::Stub::AsyncRegisterEstopEndpoint);
	}

	RegisterEstopEndpointResponse EstopClient::replaceEndpoint(const std::string &targetConfigId, bosdyn::api::EstopEndpoint &replaced, bosdyn::api::EstopEndpoint &endpoint) {
		RegisterEstopEndpointRequest request;
		assembleRequestHeader<RegisterEstopEndpointRequest>(&request);
		request.set_target_config_id(targetConfigId);
		request.mutable_new_endpoint()->CopyFrom(endpoint);
		request.mutable_target_endpoint()->CopyFrom(replaced);
		return call<RegisterEstopEndpointRequest, RegisterEstopEndpointResponse>(request, &EstopService::Stub::RegisterEstopEndpoint);
	}

	RegisterEstopEndpointResponse EstopClient::replaceEndpointAsync(const std::string &targetConfigId, bosdyn::api::EstopEndpoint &replaced, bosdyn::api::EstopEndpoint &endpoint) {
		RegisterEstopEndpointRequest request;
		assembleRequestHeader<RegisterEstopEndpointRequest>(&request);
		request.set_target_config_id(targetConfigId);
		request.mutable_new_endpoint()->CopyFrom(endpoint);
		request.mutable_target_endpoint()->CopyFrom(replaced);
		return callAsync<RegisterEstopEndpointRequest, RegisterEstopEndpointResponse>(request, &EstopService::Stub::AsyncRegisterEstopEndpoint);
	}

	DeregisterEstopEndpointResponse EstopClient::deregister(const std::string &targetConfigId, bosdyn::api::EstopEndpoint &endpoint) {
		DeregisterEstopEndpointRequest request;
		assembleRequestHeader<DeregisterEstopEndpointRequest>(&request);
		request.mutable_target_endpoint()->CopyFrom(endpoint);
		request.set_target_config_id(targetConfigId);
		return call<DeregisterEstopEndpointRequest, DeregisterEstopEndpointResponse>(request, &EstopService::Stub::DeregisterEstopEndpoint);
	}

	DeregisterEstopEndpointResponse EstopClient::deregisterAsync(const std::string &targetConfigId, bosdyn::api::EstopEndpoint &endpoint) {
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

	SetEstopConfigResponse EstopClient::setConfig(EstopConfig &config) {
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

	EstopCheckInResponse EstopClient::checkIn(EstopStopLevel &stopLevel, bosdyn::api::EstopEndpoint &endpoint, uint64_t challenge, uint64_t response, bool suppress_incorrect) {
		// TODO: implement logic for suppress_incorrect
		EstopCheckInRequest request;
		assembleRequestHeader<EstopCheckInRequest>(&request);
		request.set_challenge(challenge);
		request.set_response(response);
		request.mutable_endpoint()->CopyFrom(endpoint);
		request.set_stop_level(stopLevel);
		return call<EstopCheckInRequest, EstopCheckInResponse>(request, &EstopService::Stub::EstopCheckIn);
	}

	EstopCheckInResponse EstopClient::checkInAsync(EstopStopLevel &stopLevel, bosdyn::api::EstopEndpoint &endpoint, uint64_t challenge, uint64_t response, bool suppress_incorrect) {
		// TODO: implement logic for suppress_incorrect
		EstopCheckInRequest request;
		assembleRequestHeader<EstopCheckInRequest>(&request);
		request.set_challenge(challenge);
		request.set_response(response);
		request.mutable_endpoint()->CopyFrom(endpoint);
		request.set_stop_level(stopLevel);
		return callAsync<EstopCheckInRequest, EstopCheckInResponse>(request, &EstopService::Stub::AsyncEstopCheckIn);
	}

	int EstopThread::DEFAULT_TIME_SYNC_INTERVAL_SECS = 1;

	EstopThread::EstopThread(std::shared_ptr<EstopClient> clientPtr, std::shared_ptr<EstopEndpoint> endpoint) :
			_client(clientPtr),
			_endpoint(endpoint) {}

	EstopThread::~EstopThread() {
		endEstop();
	}

	void EstopThread::beginEstop() {
		_keepRunning = true;

		// endpoint passed in already has valid challenge / response pair
		_thread = std::shared_ptr<std::thread>(new std::thread(&EstopThread::periodicCheckIn, this));
	}

	void EstopThread::endEstop() {
		_keepRunning = false;
		if (_thread->joinable()){
			_thread->join(); // should wait for thread to stop
		}
	}

	void EstopThread::periodicCheckIn() {
		while (_keepRunning) {
			// will update challenge / response within endpoint obj
			_endpoint->allow();
			std::this_thread::sleep_for(std::chrono::seconds(DEFAULT_TIME_SYNC_INTERVAL_SECS));
			// todo: timeouts, etc.
		}
	}

};