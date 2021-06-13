#include <spot/image_client.h>

/* get_image_request(): helper function to generate an image request given the list of image requests
*/
GetImageRequest get_image_request(const std::vector<ImageRequest> &list) {
  GetImageRequest request;
  for (ImageRequest ir : list) {
    request.add_image_requests()->CopyFrom(ir);
  }
  return request;
}

ImageClient::ImageClient(const std::string &root, const std::string &server) {
  _stub = initializeNoAuthToken(server, root, "");
  _clientName = "image";
}

ListImageSourcesResponse ImageClient::listImageSources(){
  ListImageSourcesRequest request;
  assembleRequestHeader<ListImageSourcesRequest>(&request);
  return call<ListImageSourcesRequest, ListImageSourcesResponse>(request, &ImageService::Stub::ListImageSources);
}

ListImageSourcesResponse ImageClient::listImageSourcesAsync() {
  ListImageSourcesRequest request;
  assembleRequestHeader<ListImageSourcesRequest>(&request);
  return callAsync<ListImageSourcesRequest, ListImageSourcesResponse>(request, &ImageService::Stub::AsyncListImageSources);
}

GetImageResponse ImageClient::getImage(const std::vector<ImageRequest> imageRequests){
  GetImageRequest request = get_image_request(imageRequests);
  assembleRequestHeader<GetImageRequest>(&request);
  return call<GetImageRequest, GetImageResponse>(request, &ImageService::Stub::GetImage);
}

GetImageResponse ImageClient::getImageAsync(const std::vector<ImageRequest> imageRequests){
  GetImageRequest request = get_image_request(imageRequests);
  assembleRequestHeader<GetImageRequest>(&request);
  return callAsync<GetImageRequest, GetImageResponse>(request, &ImageService::Stub::AsyncGetImage);
}
