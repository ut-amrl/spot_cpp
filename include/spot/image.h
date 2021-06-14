/*
  image.h: includes client and interface for communication with the image service 
*/

#ifndef IMAGE_H
#define IMAGE_H

#include "bosdyn/api/image_service.grpc.pb.h"
#include <spot/base.h>

using bosdyn::api::ImageService;
using bosdyn::api::ImageRequest;
using bosdyn::api::ImageResponse;
using bosdyn::api::ListImageSourcesRequest;
using bosdyn::api::ListImageSourcesResponse;
using bosdyn::api::GetImageResponse;
using bosdyn::api::GetImageRequest;

class ImageClient : public BaseClient<ImageService> {
public:
	ImageClient(const std::string &authority, const std::string &token);

	GetImageResponse getImage(const std::vector<ImageRequest> imageRequests);
	GetImageResponse getImageAsync(const std::vector<ImageRequest> imageRequests);
	ListImageSourcesResponse listImageSources();
	ListImageSourcesResponse listImageSourcesAsync();
};


#endif
