# Luo_ros2_control

## 简介
- 项目定位与主要功能
- 关键特性 / 控制器列表

## 依赖
- ROS 2 Control (Jazzy)
- Eigen (v3.4)
- Boost C++ (v1.74)

## 构建
```bash
colcon build --symlink-install
```

## 运行
- 启动指令示例
```bash
ros2 launch <package> <launch_file>.launch.py
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
- `src/RosGo2Estimator`：机器人状态预测实现（依赖）

## 许可证
- 许可证类型与说明