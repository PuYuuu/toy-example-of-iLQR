# toy-example-of-iLQR
This repository implements an **C**onstrained **I**terative **L**inear **Q**uadratic **R**egulator (**CILQR**) algorithm that incorporates constraints in the environment for on-road autonomous motion planning. For more information, see [LQR与iLQR：从理论到实践【详细】](https://zhuanlan.zhihu.com/p/715102938)

<div align=center>
  <img src="./images/scenario_two_straight.gif" width="520"/>
</div>


## What's new✨

Use foxglove studio to provide better visualization and data playback. For more information, see [foxglove_guide.md](./foxglove_guide.md). [![Watch the demo](./images/demo.png)](./images/screenshot.mp4)


## 1. Prerequisites

- Tested on WSL2 Ubuntu 20.04🐧
- Other dependencies: [yamp-cpp](https://github.com/jbeder/yaml-cpp), [spdlog](https://github.com/gabime/spdlog)

## 2. Build

Clone the repository and make:

```shell
git clone https://github.com/PuYuuu/toy-example-of-iLQR.git
cd toy-example-of-iLQR
cmake -B build
cmake --build build
```

## 3. Execute examples

### 3.1 Basic examples of LQR

You can find the corresponding python script file in the `scripts` folder.

```shell
# 1. Closed-loop response of linear system under LQR controller
python scripts/0-lqr-demo.py
# 2. Path tracking using LQR
python scripts/1-lqr-pathtracking.py
# 3. Simple version of on-road motion planning by CILQR
python scripts/2-cilqr-motionplanning.py
```

### 3.2 Autonomous driving motion planning with CILQR

Find the executable file in the build folder, and specify the configuration file path through `-c` to start the program.

```shell
./build/motion_planning -c ./config/scenario_three_bend.yaml
```

<div align=center>
  <img src="./images/scenario_three_bend.gif" width="520"/>
</div>

In addition, you can manually modify the contents of the configuration file, including algorithm parameters, initial conditions, scenario information, etc., and observe the performance of CILQR in different scenarios. For example:

```shell
./build/motion_planning -c config/scenario_three_straight.yaml
```

<div align=center>
  <img src="./images/scenario_three_straight.gif" width="520"/>
</div>

And the `scenario_two_borrow.yaml` is configured for the overtaking on the opposite lane scenario:

<div align=center>
  <img src="./images/scenario_two_borrow.gif" width="520"/>
</div>
