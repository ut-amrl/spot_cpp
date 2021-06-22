#include <spot/spot_check.h>

const std::string SPOT_CHECK_CLIENT_NAME = "spot-check";

SpotCheckClient::SpotCheckClient(const std::string &authority, const std::string &token) : BaseClient(SPOT_CHECK_CLIENT_NAME, authority, token) {}

SpotCheckCommandResponse SpotCheckClient::spotCheckCommand(Lease lease, SpotCheckCommandRequest_Command command){
	SpotCheckCommandRequest request;
	assembleRequestHeader<SpotCheckCommandRequest>(&request);
	request.mutable_lease()->CopyFrom(lease);
    request.set_command(command);

	return call<SpotCheckCommandRequest, SpotCheckCommandResponse>(request, &SpotCheckService::Stub::SpotCheckCommand);
}

SpotCheckFeedbackResponse SpotCheckClient::spotCheckFeedback(){
	SpotCheckFeedbackRequest request;
	assembleRequestHeader<SpotCheckFeedbackRequest>(&request);

	return call<SpotCheckFeedbackRequest, SpotCheckFeedbackResponse>(request, &SpotCheckService::Stub::SpotCheckFeedback);
}

CameraCalibrationCommandResponse SpotCheckClient::cameraCalibration(Lease lease, CameraCalibrationCommandRequest_Command command){
	CameraCalibrationCommandRequest request;
	assembleRequestHeader<CameraCalibrationCommandRequest>(&request);
	request.mutable_lease()->CopyFrom(lease);
    request.set_command(command);

	return call<CameraCalibrationCommandRequest, CameraCalibrationCommandResponse>(request, &SpotCheckService::Stub::CameraCalibrationCommand);
}

CameraCalibrationFeedbackResponse SpotCheckClient::cameraCalibrationFeedback(){
	CameraCalibrationFeedbackRequest request;
	assembleRequestHeader<CameraCalibrationFeedbackRequest>(&request);

	return call<CameraCalibrationFeedbackRequest, CameraCalibrationFeedbackResponse>(request, &SpotCheckService::Stub::CameraCalibrationFeedback);
}