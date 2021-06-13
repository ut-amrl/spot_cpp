#ifndef ROBOT_ID_CLIENT_H
#define ROBOT_ID_CLIENT_H

#include "bosdyn/api/robot_id_service.grpc.pb.h"
#include <spot/base_client.h>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using grpc::CompletionQueue;
using grpc::ClientAsyncResponseReader;
using bosdyn::api::RobotIdService;
using bosdyn::api::RobotIdRequest;
using bosdyn::api::RobotIdResponse;
using google::protobuf::util::TimeUtil;

class RobotIdClient : public BaseClient<RobotIdService> {
public:
	RobotIdClient(const std::string& root, const std::string& server);

	RobotIdResponse getId();
	RobotIdResponse getIdAsync();
};


#endif
