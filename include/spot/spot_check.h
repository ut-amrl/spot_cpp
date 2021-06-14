/*
  spot_check.h: includes client and interface for communication with the spot-check service 
*/

#ifndef SPOT_CHECK_H
#define SPOT_CHECK_H

#include "bosdyn/api/spot/spot_check.grpc.pb.h"
#include "bosdyn/api/spot/spot_check_service.grpc.pb.h"

#include <spot/base.h>

using bosdyn::api::spot::SpotCheckService;
using bosdyn::api::spot::SpotCheckCommandRequest;
using bosdyn::api::spot::SpotCheckCommandResponse;
using bosdyn::api::spot::SpotCheckFeedbackRequest;
using bosdyn::api::spot::SpotCheckFeedbackResponse;
using bosdyn::api::spot::DepthPlaneSpotCheckResult;
using bosdyn::api::spot::PayloadCheckResult;
using bosdyn::api::spot::LoadCellSpotCheckResult;
using bosdyn::api::spot::JointKinematicCheckResult;
using bosdyn::api::spot::FootHeightCheckResult;
using bosdyn::api::spot::LegPairCheckResult;
using bosdyn::api::spot::HipRangeOfMotionResult;
using bosdyn::api::spot::CameraCalibrationCommandRequest;
using bosdyn::api::spot::CameraCalibrationCommandResponse;
using bosdyn::api::spot::CameraCalibrationFeedbackRequest;
using bosdyn::api::spot::CameraCalibrationFeedbackResponse;

const extern std::string SPOT_CHECK_CLIENT_NAME;

class SpotCheckClient : public BaseClient<SpotCheckService> {
public:
    SpotCheckClient(const std::string &authority, const std::string &token);

private:
};


#endif