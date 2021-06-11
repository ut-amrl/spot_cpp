#include <spot/time_sync.h>

TimeSyncClient::TimeSyncClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) {
    grpc::SslCredentialsOptions opts = {root, key, cert};
    stub_ = TimeSyncService::NewStub(grpc::CreateChannel(server, grpc::SslCredentials(opts)));
}

// Assembles the client's payload, sends it and presents the response back
// from the server.
TimeSyncUpdateResponse TimeSyncClient::TimeSyncUpdate(TimeSyncUpdateRequest request, const std::string& clock_identifier) {
    // Container for the data we expect from the server.
    TimeSyncUpdateResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
    request.mutable_header()->set_client_name("anything");
    
    // The actual RPC.
    Status status = stub_->TimeSyncUpdate(&context, request, &reply);

    // Act upon its status.
    if(!status.ok()) {
        std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
    }

    return reply;
}
        
TimeSyncUpdateResponse TimeSyncClient::EstablishTimeSync(const int& numRounds) {
    const std::string clock_identifier("spot_time_sync");
    TimeSyncUpdateResponse reply;
    TimeSyncUpdateRequest request;
    Duration averageSkew;
    // Keep track of best estimate overall
    TimeSyncState best;
    best.mutable_best_estimate()->mutable_round_trip_time()->set_seconds(INT64_MAX);
    int numLows = 0;

    for(int i = 0; i <= numRounds && (reply.state().status() != TimeSyncState::STATUS_UNKNOWN || i == 0); i++) {
      request.set_clock_identifier(clock_identifier);

      reply = TimeSyncUpdate(request, clock_identifier);

      // Update timestamps from previous round trip
      request.mutable_previous_round_trip()->mutable_client_rx()->CopyFrom(TimeUtil::GetCurrentTime());
      request.mutable_previous_round_trip()->mutable_client_tx()->CopyFrom(reply.header().request_header().request_timestamp());
      request.mutable_previous_round_trip()->mutable_server_tx()->CopyFrom(reply.header().response_timestamp());
      request.mutable_previous_round_trip()->mutable_server_rx()->CopyFrom(reply.header().request_received_timestamp());
      
      // Finding best time sync estimate: determine the lowest round trip delay
      // and averaging the clock skew numbers of all the samples with this minimum round trip time to reduce noise
      if(i > 0) {
        if(reply.previous_estimate().round_trip_time() == best.best_estimate().round_trip_time()) {
          averageSkew = (best.best_estimate().clock_skew() * numLows) + reply.previous_estimate().clock_skew();
          numLows++;
          averageSkew /= numLows; 
          best.mutable_best_estimate()->mutable_clock_skew()->CopyFrom(averageSkew);
          best.mutable_measurement_time()->CopyFrom(TimeUtil::GetCurrentTime());
        }
        else if(reply.previous_estimate().round_trip_time() < best.best_estimate().round_trip_time()) {
          numLows = 1;
          best.mutable_best_estimate()->mutable_round_trip_time()->CopyFrom(reply.previous_estimate().round_trip_time());
          best.mutable_best_estimate()->mutable_clock_skew()->CopyFrom(reply.previous_estimate().clock_skew());
          best.mutable_measurement_time()->CopyFrom(TimeUtil::GetCurrentTime());
        }
      }

      reply.mutable_state()->set_status(TimeSyncState::STATUS_SERVICE_NOT_READY);
    }

    reply.mutable_state()->CopyFrom(best);
    reply.mutable_state()->set_status(TimeSyncState::STATUS_OK);

    return reply;
}

TimeSyncUpdateResponse TimeSyncClient::getTimeSyncUpdate(const TimeSyncRoundTrip &previousRoundTrip, const std::string &clockIdentifier) {
	TimeSyncUpdateRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");
	request.set_clock_identifier(clockIdentifier);
	request.mutable_previous_round_trip()->CopyFrom(previousRoundTrip);

	TimeSyncUpdateResponse reply;
	ClientContext context;
	Status status = stub_->TimeSyncUpdate(&context, request, &reply);

	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
	    	std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}
	return reply;
}

TimeSyncUpdateResponse TimeSyncClient::getTimeSyncUpdateAsync(const TimeSyncRoundTrip &previousRoundTrip, const std::string &clockIdentifier) {
	TimeSyncUpdateRequest request;
  request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  request.mutable_header()->set_client_name("anything");
  request.set_clock_identifier(clockIdentifier);
	request.mutable_previous_round_trip()->CopyFrom(previousRoundTrip);

	TimeSyncUpdateResponse reply;
	ClientContext context;
	CompletionQueue cq;
	Status status;
	std::unique_ptr<ClientAsyncResponseReader<TimeSyncUpdateResponse>> rpc(
			stub_->PrepareAsyncTimeSyncUpdate(&context, request, &cq));
	rpc->StartCall();

	rpc->Finish(&reply, &status, (void *)1);
	void *got_tag;
	bool ok = false;
	GPR_ASSERT(cq.Next(&got_tag, &ok));
	GPR_ASSERT(got_tag == (void *)1);
	GPR_ASSERT(ok);

	if (status.ok()) {
		std::cout << "[SUCCEEDED]" << std::endl;
	    	std::cout << "Header timestamp info: " << reply.header().response_timestamp().seconds() << std::endl;
	} else {
		std::cout << "[FAILED]" << std::endl;
		std::cout << status.error_code() << ": " << status.error_message() << std::endl;
	}
	return reply;
}
