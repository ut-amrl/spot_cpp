/*
    common.h: common information needed throughout software
*/
#ifndef COMMON_H
#define COMMON_H

#include <grpc++/grpc++.h>
#include <google/protobuf/util/time_util.h>

#include "bosdyn/api/header.grpc.pb.h"

#include <spot/utils.h>

// variables for connecting to Spot
const std::string DEFAULT_SPOT_SERVER = "192.168.80.3";
const std::string DEFAULT_SECURE_PORT = ":443";
const std::string DEFAULT_ROOT_CERT_FILEPATH = "../src/resources/robot.pem";
// const std::string DEFAULT_ROOT_CERT = read_file(DEFAULT_ROOT_CERT_FILEPATH);
const std::string DEFAULT_ROOT_CERT = 
"-----BEGIN CERTIFICATE-----\n"
"MIIFOzCCAyOgAwIBAgIMAbE7jK/3TT5eMnR3MA0GCSqGSIb3DQEBDQUAMEkxCzAJ\n"
"BgNVBAYTAlVTMRgwFgYDVQQKEw9Cb3N0b24gRHluYW1pY3MxIDAeBgNVBAMTF0Jv\n"
"c3RvbiBEeW5hbWljcyBSb290IENBMB4XDTE4MDUwMTAwMDAwMFoXDTI5MDUwMTAw\n"
"MDAwMFowSTELMAkGA1UEBhMCVVMxGDAWBgNVBAoTD0Jvc3RvbiBEeW5hbWljczEg\n"
"MB4GA1UEAxMXQm9zdG9uIER5bmFtaWNzIFJvb3QgQ0EwggIiMA0GCSqGSIb3DQEB\n"
"AQUAA4ICDwAwggIKAoICAQDY2n0C0JNzgyMMJz/tzLdHwxEhUO6q+gX+tjRj9U4h\n"
"USlpphmJnHQwKCa53ADgT58zpJh/e+zWavTEMdYHEjSdISva5c6EhJ1EOGCYd9M/\n"
"zjFx41yvI8AgXYCLGSZUZqp8EuWo4Dj//7/gpHx278y20jSkb7G/RaZamdvt9FX1\n"
"uMQIcGpdYGPjs+qV8vCH2fnH8GoLXedHElvaWu8WC8a6ooXyk0nrTCUmS0lBwvd9\n"
"hjSU29dmJj65gvwPMbhJA4MM0tnikz/rvUlEnjuZGeqQdoH4fwIkN/uWu5ZJKyhZ\n"
"wksWaCZUXmqmLQ3sS0HkBzez7tLYSTKmjG7BbPQ7E2eFfD8cCi2wka83ahKEYL77\n"
"+3iuhfoTGcdOwm8TKD0tTDOojb/27R5XKJX7515pHfhV1U00jbZ6VpLrv3iaU28D\n"
"rgl/niL+epa7hbCmgW+oAo1QPtGrn1+eEF4QhDPScjqSHeohIaQU4rLjrRcKnfiP\n"
"PWQrxqV1Le+aJUPnqj4gOBIY8Oq61uT7k8UdIT7MivALs3+vEPJ21BYljDvMsOUm\n"
"mIzMPNo98AxAQByUYetgDEfDyObhoMcJGbadYiNdD4+foCk/8JfStMSckP2UTscS\n"
"Hq8NNmHf8ssp7Voj1t/hWh1UiRv12ii+3FSUPLH2liZVrL/zUP9MMoZVy1YogQkV\n"
"qwIDAQABoyMwITAOBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zANBgkq\n"
"hkiG9w0BAQ0FAAOCAgEAL1koxdNUVsCaDrQWGcxpO3WyuW6FVYn6G+KAsnSlqaJU\n"
"pGI77MLGrNMGCb/NkeprvrSaDMWmnfyYSYlQQIDPE1whH85hyrV1FuAy7Xt6ZSV6\n"
"oVEl83t0yViIiVuAxPBQ72682pWG1a24d9Joa2hk8oNL4MO7zNfjh6JSAy0Tsyu7\n"
"oz7rULMCCYwSzpQv3c2/gY1vEGEMxYDmpy1ym+G2MzwfJtWYmVJdrxZi3GH9i56M\n"
"wyLae8RC6QPwN+5hSy22di2VViEu59d+Pm3/HrDQwjEWUVSwP9EMEByIP+K6n+Bp\n"
"6566Utt8ezDT1poym85kqceVn8xU2aLeZelsJXNGqmLrYVdjZOC543Q8NzLnki1p\n"
"k2RL+Eld8dRe+q3aOv0HLxc8QZbWz1Bk2IlRnyZBpElAQrkyYZ4gZALoQVTLv7HC\n"
"0nLus0zaJvkfaZmwYEQnVbEFOJrQYgDbWtYFSueKzfGFX6uBY3G3gze3YMewcEuW\n"
"GrHeSPlZ2LS4lFNSONyHzT4rkf3bj9P7SnHWgvdVKO9k748StfDf/IoIqPgnUA76\n"
"Vc2K4FgvFKVAu2VMBdhdoysUbFrUF6a0e/QqPe/YRsCfTt+QoI+iZq2JezHrqzMq\n"
"//JVcAMX4mDfYcL9KhfCqHJlR30h5EmlOZaod9Oj+LvsD9NeeX2RcxlW1aURkMQ=\n"
"-----END CERTIFICATE-----";

/* standard client names used when creating clients */
const std::string AUTH_CLIENT_NAME = "auth";
const std::string DIRECTORY_REGISTRATION_CLIENT_NAME = "directory-registration";
const std::string DIRECTORY_CLIENT_NAME = "directory";
const std::string ESTOP_CLIENT_NAME = "estop";
const std::string IMAGE_CLIENT_NAME = "image";
const std::string LEASE_CLIENT_NAME = "lease";
const std::string LOCAL_GRID_CLIENT_NAME = "local-grid";
const std::string POWER_CLIENT_NAME = "power";
const std::string ROBOT_COMMAND_CLIENT_NAME = "robot-command";
const std::string ROBOT_ID_CLIENT_NAME = "robot-id";
const std::string ROBOT_STATE_CLIENT_NAME = "robot-state";
const std::string SPOT_CHECK_CLIENT_NAME = "spot-check";
const std::string TIMESYNC_CLIENT_NAME = "time-sync";
const std::string WORLD_OBJECTS_CLIENT_NAME = "world-objects";

#endif
