/*
    exception.h: contains error class definitions, from which client-specific errors inherit from
*/

#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <exception>
#include <spot/common.h>

using google::protobuf::util::TimeUtil;
using google::protobuf::Timestamp;

// base error class
class Error : public std::exception {
public:
    explicit Error(const std::string &message) : _msg(message) {}
    virtual ~Error() noexcept {}
    virtual const char *what() const noexcept { return _msg.c_str(); }
private:
    std::string _msg;
};

// triggered by server response whose rpc succeeded
template <class response_T>
class ResponseError : public Error {
public:
    ResponseError(const response_T &response, const std::string &message) : Error(message), _response(response) {}
    const int getErrorCode() const { return _response.header().error().code(); }
    const std::string getRequestReceivedTimestamp() const { return google::protobuf::util::TimeUtil::ToString(_response.header().request_received_timestamp()); }
    const std::string getResponseReceivedTimestamp() const { return google::protobuf::util::TimeUtil::ToString(_response.header().response_timestamp()); }
    const char *what() const noexcept override { 
        return ((std::string(Error::what())
                + "\nMessage: " + _response.header().error().message()
                + "\nRequest Received: " + getRequestReceivedTimestamp() 
                + "\nResponse Received: " + getResponseReceivedTimestamp()).c_str()); 
    }
private:
    const response_T &_response;
};

template<class response_T>
class LicenseError : public ResponseError<response_T> {

};

// error during rp call
template <class request_T>
class RpcError : public Error {
public:
    RpcError(const request_T &request, const std::string &message) : Error(message), _request(request) {}
    const std::string getRequestTimestamp() const { return google::protobuf::util::TimeUtil::ToString(_request.header().request_timestamp()); }
    const std::string getClientName() const { return _request.header().client_name(); }
    const char *what() const noexcept override { 
        return ((std::string(Error::what())
                + "\nClient Name: " + getClientName()
                + "\nRequest Received: " + getRequestTimestamp()).c_str()); 
    }
private:
    const request_T &_request;
};

#endif