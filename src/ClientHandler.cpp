#include <spot/ClientHandler.h>

ClientHandler::ClientHandler(std::string& hostname, std::string& cert) : 
	_authClient(cert, hostname),
	_estopClient(cert, hostname),
	_imageClient(cert, hostname),
	_leaseClient(cert, hostname),
	_powerClient(cert, hostname),
	_robotCommandClient(cert, hostname),
	_robotIdClient(cert, hostname),
	_robotStateClient(cert, hostname),
	// _spotCheckClient(cert, hostname),
	_timeSyncClient(cert, hostname){
	
}

AuthClient& ClientHandler::authClient(){
	return _authClient;
}

EstopClient& ClientHandler::estopClient(){
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
	return _robotStateClient;
}

// SpotCheckClient& ClientHandler::spotCheckClient(){
// 	return &_spotCheckClient;
// }

TimeSyncClient& ClientHandler::timeSyncClient(){
	return _timeSyncClient;
}

