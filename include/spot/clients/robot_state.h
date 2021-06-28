/*
  robot_state.h: includes client and interface for communication with the robot-state service 
*/

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "bosdyn/api/robot_state_service.grpc.pb.h"
#include "bosdyn/api/geometry.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"

#include <spot/clients/base.h>

using bosdyn::api::RobotStateService;
using bosdyn::api::RobotStateRequest;
using bosdyn::api::RobotStateResponse;
using bosdyn::api::RobotState;
using bosdyn::api::PowerState;
using bosdyn::api::CommsState;
using bosdyn::api::SystemFaultState;
using bosdyn::api::EStopState;
using bosdyn::api::KinematicState;
using bosdyn::api::BehaviorFaultState; 
using bosdyn::api::FootState;
using bosdyn::api::RobotMetrics;
using bosdyn::api::RobotMetricsRequest;
using bosdyn::api::RobotMetricsResponse;
using bosdyn::api::HardwareConfiguration;
using bosdyn::api::RobotHardwareConfigurationRequest;
using bosdyn::api::RobotHardwareConfigurationResponse;
using bosdyn::api::RobotLinkModelRequest;
using bosdyn::api::RobotLinkModelResponse;

namespace ClientLayer {

  class RobotState {
  public:
    RobotState();
  private:
  };

  class RobotMetric {
  public:
    RobotMetric();
  private:
  };

  class RobotHardwareConfiguration {
  public:
    RobotHardwareConfiguration();
  private:
  };

  class RobotStateClient : public BaseClient<RobotStateService> { 
  public:
    RobotStateClient(const std::string &authority, const std::string &token);
    
    RobotStateResponse getRobotState();
    RobotStateResponse getRobotStateAsync();
    RobotMetricsResponse getRobotMetrics();
    RobotMetricsResponse getRobotMetricsAsync();
    RobotHardwareConfigurationResponse getRobotHardwareConfiguration();
    RobotHardwareConfigurationResponse getRobotHardwareConfigurationAsync();
    RobotLinkModelResponse getRobotLinkModel(const std::string &linkName);
    RobotLinkModelResponse getRobotLinkModelAsync(const std::string &linkName);
  };

};

#endif
