# RESC Library

RESC Library (resclib) is a part of the RESC-pilot project.

It can be used for RL training(rescrl) and provides interfaces for ROS(rescros).

## Installation

**Note:** tested on Ubuntu 20.04 only

### Prerequisites

* pybind11
* eigen3

### Install

configure and build the library using CMake:

```shell
cd /path_to_resclib/resclib
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

install the library to the system:

```shell
sudo make install
```

and you can also uninstall it using:

```shell
sudo make uninstall
```
