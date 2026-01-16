# simMotor
## joint_state_topic -> 关节状态订阅话题
映射到Gazebo中JointTrajectoryController对应的话题 \

## joint_command_topic -> 关节命令发布话题
映射到Gazebo中JointStatePublisher对应的话题

# simIMU
## imu_topic -> imu消息订阅话题 \
映射到Gazebo中gz-sim-imu-system对应的话题
## imu_name -> imu传感器名称
对应ros2 Control的URDF文件中，sensor的名称

# simContact
contacts_topic_LF -> 左前足接触发布话题 \
contacts_topic_RF -> 右前足接触发布话题 \
contacts_topic_LF -> 左后足接触发布话题 \
contacts_topic_LF -> 右后足接触发布话题

保证Gazebo中的关节顺序为 \
"LF_HAA", "LF_HFE", "LF_KFE" \
"LH_HAA", "LH_HFE", "LH_KFE" \
"RF_HAA", "RF_HFE", "RF_KFE" \
"RH_HAA", "RH_HFE", "RH_KFE"