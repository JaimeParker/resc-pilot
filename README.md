# ReSC-pilot

A search-to-control reinforcement learning (RL) based planning framework for quadrotor agile flight.

**News**:

* **June 28, 2025**: accepted by IEEE Robotics and Automation Letters (RA-L)
* **July 23, 2025**: published in IEEE RA-L, available on [IEEE Xplore](https://ieeexplore.ieee.org/document/11091396).
* Video demo available on [YouTube](https://www.youtube.com/watch?v=efjKPxH79qk) and [Bilibili](https://www.bilibili.com/video/BV13Z87zaE4x).

![rviz sim](assets/rvizsim.gif)


NOTE: **do not** use the model on your real drone directly! The model we released is not the one in the paper, for test purpose only.

## 1. Setup and Config

### Prerequisites

1. Our software is developed and tested in Ubuntu 20.04 (ROS noetic). Follow documents to install [ROS Noetic](https://wiki.ros.org/noetic/Installation).
2. We use [**Libtorch**](https://pytorch.org/cppdocs/installing.html) for model inference, which can be downloaded from [pytorch]([PyTorch](https://pytorch.org/)), depending on the version of [CUDA](https://developer.nvidia.com/cuda-downloads) or CPU instead. For our model, cpu version is enough. If you choose to use GPU version, please handle the CUDA version in CMakeLists.txt.
3. Basic 3rd parties including [Eigen3.3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
4. pybind11 is required for building the RESC Library (resclib), you can install it via `pip install "pybind11[global]"` or build it from source. pybind11 is not required if without RL training. If you don't need RL training, you can revise CMakeLists.txt in resclib to remove the pybind11 dependency.

### Build on ROS

First, build the RESC Library (resclib):

```bash
cd {YOUR_WORKSPACE}/src
git clone https://github.com/JaimeParker/resc-pilot.git
cd resc-pilot/resclib
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
sudo make install
```

`FindTorch.cmake` is developing, since the package is decompressed in a custom directory, we need to set the path manually currently.

In the [rescros/backend_optimizer/CMakeLists.txt](rescros/backend_optimizer/CMakeLists.txt) line 20, add

```cmake
set(Torch_DIR ~/3rdParty/libtorch/share/cmake/Torch)
```

and change the path to your libtorch directory.

```bash
cd {YOUR_WORKSPACE}
catkin_make
```

## 2. Run

use random obstacles for testing:

```bash
roslaunch plan_admin sim250.launch
```

# Acknowledgements

We would like to thank the following individuals and organizations for their valuable contributions and support:

* **[Prof. Dong](https://www.aero.sjtu.edu.cn/szdw/szml/53)** for discussions, guidance, funding and equipment support

and

* **[Wenxuan Gao](https://github.com/JyzGao)** for the support on sim2real transfer debug (PX4), RL observation discussion, and assistance with real-world experiments (May to Nov, 2024).
* **[Dr. Yinshuai Sun](https://github.com/YinshuaiSun)** for support on drone design, real-world experiments, thrust measurement, torque coefficient measurement experiments, system identification experiments, and paper revising guidance (**Apr 2024 to Jun 2025**).
* **[Dr. Yunlong Song](https://github.com/yun-long)** for the discussions and support on sim2real transfer and RL environment design (May to Aug, 2024).
* **[Baiyang Li](https://github.com/goth42378)** for support on PX4-Autopilot discussion and co-debug in rate controller, mixer and control allocator (Nov and Dec, 2024).
* **[Zeshuai Chen](https://github.com/zschen879)** for discussions on UAV configuration, motion capture communication setup, IMU integration, as well as plotting and video creation.
* **[Dr. Tao Cui](https://github.com/cuitao-sjtu)** for assistance with thrust measurement experiments and early discussions on RL reward functions (Mar to May, 2024).
* **[Hongzheng Zhu](https://github.com/PiggerHustle)** for assistance with motor modeling and real-world experiments (Sept and Dec, 2024).
* **[Yubo Dong](https://github.com/clayearth)** for discussions on RL observation space, SB3 preserving steps and hyperparameter-tuning trails (Mar to May, 2024).
* **[Jia Xu](https://gitee.com/xuchengbei)** for discussions on UAV configuration.
* **Yufan Zhou** for assistance with real-world experiments (Dec, 2024).
* **Jingyi Tu** for support with torque coefficient measurement experiments (Oct, 2024).
* T-Motor for providing the T-Motor F60Pro Kv2550 motor parameters.
* Nokov for supporting motion capture equipment.

The project is inspired by [flightmare](https://github.com/uzh-rpg/flightmare) and [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner).

Plotting inspired by [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner/tree/master).

# License

The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

# Maintenance

I'm still working on improving code reliability and future features. If you have any questions or suggestions, please feel free to open an issue on GitHub.

For any technical issues, please contact me at (zhliu25@sjtu.edu.cn).

The training code will be released in the future.