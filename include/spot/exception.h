/*
    exception.h: contains error class definitions, from which client-specific errors inherit from
*/

#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <exception>

// base exception
class Error : public std::exception {

};

// triggered by server response whose rpc succeeded
class ResponseError : public Error {

};

class InvalidRequestError : public ResponseError {

};

class LeaseUseError : public ResponseError {

};

class LicenseError : public ResponseError {

};

// service encountered an error
class ServerError : public ResponseError {

};

class InternalServiceError : public ServerError {

};

class UnsetStatusError : public ServerError {

};

// error during rp call
class RpcError : public Error {

};

class InvalidAppTokenError : public RpcError {

};

class InvalidClientCertificateError : public RpcError {

};

class NonexistentAuthorityError : public RpcError {

};

class UnauthenticatedError : public RpcError {

};

class UnableToConnectToRobotError : public RpcError {

};

// time sync required, but not established yet
class TimeSyncRequired : public Error {

};

#endif