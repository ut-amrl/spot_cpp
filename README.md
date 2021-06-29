# Spot C++ SDK and ROS API
This repository hosts a C++ version of the Boston Dynamics Spot SDK, as well as the software interface that allows ROS integration with Spot via gRPC calls using the C++ SDK.

# Dependencies
gRPC C++

# Installation
1. Clone this repo
2. <a href="https://grpc.io/docs/languages/cpp/quickstart/">Install gRPC</a>

# Building the Library
1. Run ./generate_protos to generate source code for the .proto files provided from Boston Dynamics.

```
$./generate_protos
```

2. Create build folder and compile using CMake.
Starting from parent directory:

```
mkdir -p cmake/build && cd cmake/build
```
```
cmake ../..
```
```
make
```
```
[sudo] make install
```
# Building Examples
1. CD into a certain example in the examples directory, as an example 'teleop.'
```
cd examples/teleop
```

2. Create build folder and compile using CMAKE.
```
mkdir -p cmake/build && cmake/build
```
```
make
```

# SDK Architecture
The SDK is organized based on <a href="https://dev.bostondynamics.com/docs/concepts/readme">this</a>, where layers are namespaces and clients and their containers are objects. 

