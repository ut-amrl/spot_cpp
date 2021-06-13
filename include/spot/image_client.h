#ifndef IMAGE_CLIENT_H
#define IMAGE_CLIENT_H

#include "bosdyn/api/image_service.grpc.pb.h"
#include <spot/base_client.h>

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

class ImageClient : public BaseClient<ImageService> {
public:
	ImageClient(const std::string &root, const std::string &server);

	GetImageResponse getImage(const std::vector<ImageRequest> imageRequests);
	GetImageResponse getImageAsync(const std::vector<ImageRequest> imageRequests);
	ListImageSourcesResponse listImageSources();
	ListImageSourcesResponse listImageSourcesAsync();
};


#endif
