#ifndef CLIENT_HANDLER_H
#define CLIENT_HANDLER_H

#include <spot/auth_client.h>
#include <spot/directory_client.h>
#include <spot/estop_client.h>
#include <spot/image_client.h>
#include <spot/power_client.h>
#include <spot/robot_command_client.h>
#include <spot/robot_id_client.h>
#include <spot/robot_state_client.h>
#include <spot/spot_check_client.h>
#include <spot/time_sync_client.h>
#include <spot/lease_client.h>

class ClientHandler{
public:
	ClientHandler(const std::string& hostname, const std::string& cert);

	void setAuthToken(std::string token);

	AuthClient& authClient();
	DirectoryClient& directoryClient();
	EstopClient& estopClient();
	ImageClient& imageClient();
	LeaseClient& leaseClient();
	PowerClient& powerClient();
	RobotCommandClient& robotCommandClient();
	RobotIdClient& robotIdClient();
	RobotStateClient& robotStateClient();
	// SpotCheckClient& spotCheckClient();
	TimeSyncClient& timeSyncClient();

private:
	template <class client_T>
	void ensureAuthorization(client_T* client);

	AuthClient _authClient;
	DirectoryClient _directoryClient;
	EstopClient _estopClient;
	ImageClient _imageClient;
	LeaseClient _leaseClient;
	PowerClient _powerClient;
	RobotCommandClient _robotCommandClient;
	RobotIdClient _robotIdClient;
	RobotStateClient _robotStateClient;
	// SpotCheckClient _spotCheckClient;
	TimeSyncClient _timeSyncClient;

	// Keeps track of which clients have been initialized with an auth token
	// <Client Name, Authority>
	std::map<std::string, std::string> _authedClients;

	// Hostname
	std::string _hostname;
	// SSL Certificate
	std::string _cert;
	// Authorization Token
	std::string _authToken;
};

#endif
