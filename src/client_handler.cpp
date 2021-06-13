#include <spot/client_handler.h>


ClientHandler::ClientHandler(const std::string& hostname, const std::string& cert) : 
	_authClient(cert, hostname),
	_directoryClient(cert, hostname),
	_estopClient(cert, hostname),
	_imageClient(cert, hostname),
	_leaseClient(cert, hostname),
	_powerClient(cert, hostname),
	_robotCommandClient(cert, hostname),
	_robotIdClient(cert, hostname),
	_robotStateClient(cert, hostname),
	// _spotCheckClient(cert, hostname),
	_timeSyncClient(cert, hostname),
	_authedClients(){
	
	_authedClients.insert(std::pair<std::string, std::string>("auth", "auth.spot.robot"));

	_hostname = hostname;
	_cert = cert;
	_authToken = ""; // Initially set to empty to represent the lack of an auth token
}

template <class client_T>
void ClientHandler::ensureAuthorization(client_T* client){
	if(_authToken.empty()){
		std::cout << "Failed to authenticate " << client->getClientName() << " client, no auth token" << std::endl;
		return;
	}
	if(_authedClients.count("directory") <= 0){
		_directoryClient.authenticateStub(_authToken, "api.spot.robot");
		_authedClients.insert(std::pair<std::string, std::string>("directory", "api.spot.robot"));
	}
	if(_authedClients.count(client->getClientName()) <= 0){
		std::string authority = _directoryClient.getEntry(client->getClientName()).service_entry().authority();
		if(authority.find('.') != std::string::npos){
			authority = authority.substr(0, authority.find('.'));
		}
		authority += ".spot.robot";
		_authedClients.insert(std::pair<std::string, std::string>(client->getClientName(), authority));
		client->authenticateStub(_authToken, authority);
	}
}

void ClientHandler::setAuthToken(std::string token){
	_authToken = token;
}

AuthClient& ClientHandler::authClient(){
	return _authClient;
}

DirectoryClient& ClientHandler::directoryClient(){
	ensureAuthorization<DirectoryClient>(&_directoryClient);
	return _directoryClient;
}

EstopClient& ClientHandler::estopClient(){
	ensureAuthorization<EstopClient>(&_estopClient);
	return _estopClient;
}

ImageClient& ClientHandler::imageClient(){
	return _imageClient;
}

LeaseClient& ClientHandler::leaseClient(){
	return _leaseClient;
}

PowerClient& ClientHandler::powerClient(){
	return _powerClient;
}

RobotCommandClient& ClientHandler::robotCommandClient(){
	return _robotCommandClient;
}

RobotIdClient& ClientHandler::robotIdClient(){
	return _robotIdClient;
}

RobotStateClient& ClientHandler::robotStateClient(){
	ensureAuthorization<RobotStateClient>(&_robotStateClient);
	return _robotStateClient;
}

// SpotCheckClient& ClientHandler::spotCheckClient(){
// 	return &_spotCheckClient;
// }

TimeSyncClient& ClientHandler::timeSyncClient(){
	return _timeSyncClient;
}



