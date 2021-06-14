#include <spot/robot_id.h>

const static std::string CLIENT_NAME = "robot-id";
const static std::string AUTHORITY = "id.spot.robot";
const static std::string TOKEN = "";

RobotIdClient::RobotIdClient() : BaseClient(CLIENT_NAME, AUTHORITY, TOKEN) {}

RobotIdResponse RobotIdClient::getId(){
  RobotIdRequest request;
  assembleRequestHeader<RobotIdRequest>(&request);
  return call<RobotIdRequest, RobotIdResponse>(request, &RobotIdService::Stub::GetRobotId);
}


RobotIdResponse RobotIdClient::getIdAsync(){
  RobotIdRequest request;
  assembleRequestHeader<RobotIdRequest>(&request);
  return callAsync<RobotIdRequest, RobotIdResponse>(request, &RobotIdService::Stub::AsyncGetRobotId);
}
