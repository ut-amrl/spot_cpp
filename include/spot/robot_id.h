/*
  robot_id.h: includes client and interface for communication with the robot-id service 
*/

#ifndef ROBOT_ID_H
#define ROBOT_ID_H

#include "bosdyn/api/robot_id_service.grpc.pb.h"
#include <spot/base.h>

using bosdyn::api::RobotId;
using bosdyn::api::RobotIdService;
using bosdyn::api::RobotIdRequest;
using bosdyn::api::RobotIdResponse;

const extern std::string ROBOT_ID_CLIENT_NAME;

class RobotIdClient : public BaseClient<RobotIdService> {
public:
	RobotIdClient();

	RobotIdResponse getId();
	RobotIdResponse getIdAsync();
};

#endif
