#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>
#include <grpc++/heal_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include <spot/RobotIdServiceImpl.h>

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using grpc::ServerCredentials;

// copied
void read(const std::string& filename, std::string& data) {
  std::ifstream file(filename.c_str(), std::ios::in);
  if (file.is_open())
  {
    std::stringstream ss;
    ss << file.rdbuf();
    file.close();
    data = ss.str();
  }
  return;
}

void runServer(std::string server_address) {
    // include service
    RobotIdServiceImpl robot_id_service;

    // grpc stuff
    grpc::EnableDefaultHealthCheckService(true);
    grpc::reflection::InitProtoReflectionServerBuilderPlugin();
    ServerBuilder builder;

    // cert stuff (todo using robot.pem)
    
    // add on address with creds
    builder.AddListeningPort(server_address, creds);      

    // register service
    builder.RegisterService(&robot_id_service);

    // assemble server
    std::unique_ptr<Server> server(builder.BuildAndStart());
    std::cout << "Server listening on " << server_address << std::endl;    
}

int main(int argc, char *argv[]) {
    grpc_init();

    // start server w/ spot ip
    std::string server_address = "192.168.80.3";
    runServer(server_address);
}
