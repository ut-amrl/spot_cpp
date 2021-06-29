# Spot C++ SDK and ROS API
This repository hosts a C++ version of the Boston Dynamics Spot SDK, as well as the software interface that allows ROS integration with Spot via gRPC calls.

# Dependencies
gRPC C++
sigc++-3.0
gtk4
glibmm-2.68
cairomm-1.16
pangomm-2.48
pkg-config
glib-2.0
pango
cairo
gdk-pixbuf-2.0
graphene-1.0

# Installation
1. Clone this repo
2. <a href="https://grpc.io/docs/languages/cpp/quickstart/">Install gRPC</a>

# Building
1. Run ./generate_protos to generate source code for .proto files provided from Boston Dynamics.

```
$./generate_protos
```

2. Create build folder and compile using CMake.
Starting from parent directory:

```
mkdir build && cd build
```
```
cmake ..
```
```
make
```
