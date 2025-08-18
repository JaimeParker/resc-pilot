# rescros

ROS package for Resc-Pilot.

## 1. Setup and Config

### Prerequisites

1. Our software is developed and tested in Ubuntu 20.04 (ROS noetic). Follow documents to install [ROS Noetic](https://wiki.ros.org/noetic/Installation).
2. We use [**Libtorch**](https://pytorch.org/cppdocs/installing.html) for model inference, which can be downloaded from [pytorch]([PyTorch](https://pytorch.org/)), depending on the version of [CUDA](https://developer.nvidia.com/cuda-downloads) or CPU instead. For our model, cpu version is enough.
3. Basic 3rd parties including [Eigen3.3](https://eigen.tuxfamily.org/index.php?title=Main_Page)

'FindTorch.cmake' is developing, since the package is decompressed in a custom directory, we need to set the path manually currently.

In the [backend_optimizer/CMakeLists.txt](backend_optimizer/CMakeLists.txt) line 20, add

```cmake
set(Torch_DIR ~/3rdParty/libtorch/share/cmake/Torch)
```

and change the path to your libtorch directory.

### Build on ROS

```bash
cd {YOUR_WORKSPACE}
catkin_make
```

## 2. Run

use random obstacles for testing

```bash
roslaunch plan_admin sim250.launch
```