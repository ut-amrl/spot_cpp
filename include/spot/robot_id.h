#ifndef ROBOT_ID_H
#define ROBOT_ID_H

#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>


#include "bosdyn/api/robot_id_service.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"
#include <google/protobuf/util/time_util.h>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using grpc::CompletionQueue;
using grpc::ClientAsyncResponseReader;
using bosdyn::api::RobotIdService;
using bosdyn::api::RobotIdRequest;
using bosdyn::api::RobotIdResponse;
using google::protobuf::util::TimeUtil;

class RobotIdClient{
public:
	RobotIdClient(const std::string& root, const std::string& server);

	RobotIdResponse getId();
	RobotIdResponse getIdAsync();
private:
	std::unique_ptr<RobotIdService::Stub> stub_;
};


#endif
