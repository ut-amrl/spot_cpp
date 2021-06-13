#include <spot/robot_id_client.h>

RobotIdClient::RobotIdClient(const std::string& root, const std::string& server) {
    _stub = initializeNoAuthToken(server, root, "id.spot.robot");
    _clientName = "id";
}

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
