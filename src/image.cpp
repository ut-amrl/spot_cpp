#include <spot_comm/image.h>

Image::Image(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) {
  grpc::SslCredentialsOptions opts = {root, key, cert};
  stub_ = ImageService::NewStub(grpc::CreateChannel(server, grpc::SslCredentials(opts)));
}

ListImageSourcesResponse Image::listImageSources(){
  // Data we are sending to the server.
  ListImageSourcesRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  
  // Container for the data we expect from the server.
  ListImageSourcesResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->ListImageSources(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Success" << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message() << std::endl;
  }

  return reply;
}


GetImageResponse Image::getImage(std::vector<ImageRequest> imageRequests){
  // Data we are sending to the server.
  GetImageRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  for(ImageRequest ir : imageRequests){
	  request.add_image_requests()->CopyFrom(ir);
  }
  
  // Container for the data we expect from the server.
  GetImageResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  Status status = stub_->GetImage(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    std::cout << "Success" << std::endl;
  } else {
    std::cout << status.error_code() << ": " << status.error_message() << std::endl;
  }

  return reply;
}


GetImageResponse Image::getImageAsync(std::vector<ImageRequest> imageRequests){
  // Data we are sending to the server.
  GetImageRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  for(ImageRequest ir : imageRequests){
	  request.add_image_requests()->CopyFrom(ir);
  }
  
  // Container for the data we expect from the server.
  GetImageResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  ClientContext context;

  // The actual RPC.
  CompletionQueue cq;
  std::unique_ptr<ClientAsyncResponseReader<GetImageResponse>> rpc(stub_->AsyncGetImage(&context, request, &cq));

  Status status;
  rpc->Finish(&reply, &status, (void*)1);

  void* got_tag;
  bool ok = false;
  if(!cq.Next(&got_tag, &ok)){
    std::cout << "Error: completion queue is fully drained" << std::endl;
  }
  else{
    if (ok && got_tag == (void*)1) {
      // Act upon its status.
      if (status.ok()) {
        std::cout << "Success" << std::endl;
      } else {
        std::cout << status.error_code() << ": " << status.error_message() << std::endl;
      }
    }
    else{
      std::cout << "Error: next value in completion queue does not match" << std::endl;
    }
  }

  return reply;
}