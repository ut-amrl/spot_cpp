#include <spot/clients/robot_id.h>

const static std::string AUTHORITY = "id.spot.robot";
const static std::string TOKEN = "";

namespace ClientLayer {

  RobotIdClient::RobotIdClient() : BaseClient(ROBOT_ID_CLIENT_NAME, AUTHORITY, TOKEN) {}

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

};