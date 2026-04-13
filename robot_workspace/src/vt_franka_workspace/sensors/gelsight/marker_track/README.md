# Marker Tracking Source Code

## Introduction

In this directory (./reactive_diffusion_policy/real_world/publisher/marker_track), we provide the source code and compilation files for find_marker.so (located at ./reactive_diffusion_policy/real_world/publisher/lib/find_marker.so). 

While our precompiled find_marker.so suffices for most scenarios, we provide the source code here to accommodate platform-specific compilation requirements and enable user customization.

## Requirements

* opencv
* pybind11
* numpy

```
pip3 install pybind11 numpy opencv-python
```

## Build from source

Enter the current directory

```
cd ./reactive_diffusion_policy/real_world/publisher/marker_track
```

Modify **makefile** based on your own platform

Make the project

```
make
```

Now **find_marker.so** should be under directory ./reactive_diffusion_policy/real_world/publisher/marker_track/lib, simply **replace** it with the original find_marker.so(located at ./reactive_diffusion_policy/real_world/publisher/lib/find_marker.so). 

## Third-party Components

The marker tracking component of this project is based on:

    [tracking] by Shaoxiong Wang

        Source: https://github.com/Gelsight/tracking

        License: MIT
