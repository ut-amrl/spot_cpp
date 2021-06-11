#ifndef IMAGE_H
#define IMAGE_H

#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>


#include "bosdyn/api/image_service.grpc.pb.h"
#include "bosdyn/api/header.grpc.pb.h"
#include <google/protobuf/util/time_util.h>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using grpc::CompletionQueue;
using grpc::ClientAsyncResponseReader;
using bosdyn::api::ImageService;
using bosdyn::api::ImageRequest;
using bosdyn::api::ImageResponse;
using bosdyn::api::ListImageSourcesRequest;
using bosdyn::api::ListImageSourcesResponse;
using bosdyn::api::GetImageResponse;
using bosdyn::api::GetImageRequest;
using google::protobuf::util::TimeUtil;

class ImageClient {
public:
	ImageClient(const std::string &root, const std::string &server);

	GetImageResponse getImage(std::vector<ImageRequest> imageRequests);
	GetImageResponse getImageAsync(std::vector<ImageRequest> imageRequests);
	ListImageSourcesResponse listImageSources();
	ListImageSourcesResponse listImageSourcesAsync();
private:
	std::unique_ptr<ImageService::Stub> stub_;
};


#endif
