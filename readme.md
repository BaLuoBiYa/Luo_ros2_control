# Luo_ros2_control

## 简介
- 项目定位与主要功能
- 关键特性 / 控制器列表

## 依赖
- ROS 2 Control (Jazzy)

## 克隆
```bash
cd ~/ws
mkdir src
cd src
git clone --recurse-submodules https://github.com/BaLuoBiYa/Luo_ros2_control.git
cd ..
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-dev \
  pybind11-dev \
  libeigen3-dev \
  libboost-all-dev \
  libglpk-dev \
  libgmp-dev \
  libmpfr-dev \
  libcgal-dev \
  libopencv-dev \
  libpcl-dev \
  liburdfdom-dev \
  libpcap-dev \
  libgz-sim8-dev \
  ros-jazzy-eigen3-cmake-module \
  ros-jazzy-hpp-fcl \
  ros-jazzy-grid-map \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-coal \
  ros-jazzy-pinocchio \
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 构建
```bash
touch src/ocs2/ocs2_robotic_examples/COLCON_IGNORE
colcon build --symlink-install
export GZ_SIM_RESOURCE_PATH=src/simulation
source install/setup.bash
```

## 运行
```bash
ros2 launch legged_bringup gazebo.launch.py
启动gazebo仿真

ros2 launch legged_bringup control_loop.launch.py
启动ocs2控制器 或者
ros2 launch legged_bringup test.launch.py
启动基于话题的关节控制器
```

## 目录结构
- `src/legged_controllers`：MRT和WBC ROS2 Control接口
- `src/legged_description`：机器人描述以及MPC参数
- `src/legged_estimator`：  机器人状态预测ROS2 Control接口
- `src/legged_hardware`：机器人硬件接口
- `src/ocs2_legged_robot`：基于ocs2的四足机器人特化配置
- `src/ocs2_legged_robot_ros`：ocs2到ros的接口
- `src/ocs2`：ocs2的核心实现
- `src/qpoases_colcon`：qpOASES的ros装饰（依赖）
- `src/simulation`: Gazebo仿真描述文件

## 许可证
- 许可证类型与说明
