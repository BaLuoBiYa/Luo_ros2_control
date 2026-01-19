# Luo_ros2_control

## 简介
- 项目定位与主要功能
- 关键特性 / 控制器列表

## 依赖
- ROS 2 Control (Jazzy)

## 克隆
```bash
apt update
git clone --recurse-submodules https://github.com/BaLuoBiYa/Luo_ros2_control.git
rosdep install --from-paths src --ignore-src -r -y
```

## 构建
```bash
colcon build --packages-up-to legged_controllers legged_hardware legged_bringup --symlink-install
export GZ_SIM_RESOURCE_PATH=path_to_ws/src/simulation
```

## 运行
```bash
ros2 launch legged_bringup gazebo.launch.py
启动gazebo仿真
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
- `src/ocs2_ros2`：ocs2的核心实现
- `src/qpoases_colcon`：qpOASES的ros装饰（依赖）
- `src/simulation`: Gazebo仿真描述文件

## 许可证
- 许可证类型与说明



sudo apt-get install libpcap-dev