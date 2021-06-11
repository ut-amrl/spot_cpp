#ifndef CLIENT_HANDLER_H
#define CLIENT_HANDLER_H

#include <spot/auth.h>
#include <spot/estop.h>
#include <spot/image.h>
#include <spot/power.h>
#include <spot/robot_command.h>
#include <spot/robot_id.h>
#include <spot/robot_state.h>
#include <spot/spot_check.h>
#include <spot/time_sync.h>
#include <spot/lease.h>

class ClientHandler{
public:
	ClientHandler(const std::string& hostname, const std::string& cert);

	AuthClient& authClient();
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
	AuthClient _authClient;
	EstopClient _estopClient;
	ImageClient _imageClient;
	LeaseClient _leaseClient;
	PowerClient _powerClient;
	RobotCommandClient _robotCommandClient;
	RobotIdClient _robotIdClient;
	RobotStateClient _robotStateClient;
	// SpotCheckClient _spotCheckClient;
	TimeSyncClient _timeSyncClient;
};

#endif
