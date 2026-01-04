#include "legged_estimator/legged_estimator.hpp"

using namespace legged_robot;

controller_interface::CallbackReturn legged_estimator::on_init()
{
    joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
    if (joint_names_.size() != 12u) {
        RCLCPP_ERROR(
            get_node()->get_logger(), "Expected 12 joint names, got %zu", joint_names_.size());
        return CallbackReturn::ERROR;
    }

    tipforce_names_ = auto_declare<std::vector<std::string>>("tipforces", {});
    if (tipforce_names_.size() != 4u) {
        RCLCPP_ERROR(
            get_node()->get_logger(), "Expected 4 tipforce names, got %zu", tipforce_names_.size());
        return CallbackReturn::ERROR;
    }

    imu_name_ = auto_declare<std::string>("imu_name", "imu");

    pub_odom_topic    = auto_declare<std::string>("pub_odom_topic", "NoYamlRead/Odom");
    pub_odom_2d_topic = auto_declare<std::string>("pub_odom2d_topic", "NoYamlRead/Odom_2D");

    odom_frame_id    = auto_declare<std::string>("odom_frame", "odom");
    child_frame_id   = auto_declare<std::string>("base_frame", "base_link");
    child_frame_2d_id= auto_declare<std::string>("base_frame_2d", "base_link_2D");

    imu_data_enable  = auto_declare<bool>("imu_data_enable", false);
    if (!imu_data_enable) {
        RCLCPP_ERROR(get_node()->get_logger(), "IMU MUST be enabled for legged estimator!");
        return CallbackReturn::ERROR;
    }

    leg_pos_enable   = auto_declare<bool>("leg_pos_enable", true);
    leg_ori_enable   = auto_declare<bool>("leg_ori_enable", true);
    publish_odom_enable = auto_declare<bool>("publish_odom_enable", true);

    leg_ori_init_weight = auto_declare<double>("leg_ori_init_weight", 0.001);
    leg_ori_time_wight  = auto_declare<double>("leg_ori_time_wight", 100.0);
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn legged_estimator::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    for (int i = 0; i < 2; ++i) {
        auto * ptr = new EstimatorPortN;
        StateSpaceModel_Go2_Initialization(ptr);
        StateSpaceModel_Go2_Sensors.emplace_back(ptr);
    }

    Sensor_IMUAcc      = std::make_shared<DataFusion::SensorIMUAcc>     (StateSpaceModel_Go2_Sensors[0]);
    Sensor_IMUMagGyro  = std::make_shared<DataFusion::SensorIMUMagGyro> (StateSpaceModel_Go2_Sensors[1]);
    Sensor_LegsPos     = std::make_shared<DataFusion::SensorLegsPos>    (StateSpaceModel_Go2_Sensors[0]);
    Sensor_LegsOri     = std::make_shared<DataFusion::SensorLegsOri>    (StateSpaceModel_Go2_Sensors[1]);
    Sensor_LegsOri->SetLegsPosRef(Sensor_LegsPos.get());
    
    for (int i = 0; i < 9; ++i) {
        position_correct[i] = 0;
        orientation_correct[i] = 0;
    }

    if(!(leg_ori_init_weight>-1.0 && leg_ori_init_weight<=1.0))
    {
        leg_ori_init_weight = 0.001;
        RCLCPP_INFO(get_node()->get_logger(), "leg_ori_init_weight to %lf",leg_ori_init_weight);
    }
    if(!(leg_ori_time_wight>0.0 && leg_ori_time_wight<=1000000.0))
    {
        leg_ori_time_wight = 100.0;
        RCLCPP_INFO(get_node()->get_logger(), "leg_ori_time_wight to %lf", leg_ori_time_wight);
    }

    rclcpp::Parameter kin_param;
    if (get_node()->get_parameter("kinematic_params", kin_param)) 
    {
        auto v = kin_param.as_double_array();  // std::vector<double>
        if (v.size() == 4u * 13u) {
            // 用 Eigen::Map 把一维数组映射成 4×13 矩阵（行优先）
            Eigen::Map<const Eigen::Matrix<double, 4, 13, Eigen::RowMajor>> kin_map(v.data());
            Sensor_LegsPos->KinematicParams = kin_map;

            // 用新的 KinematicParams 更新几何长度（若你想保持默认值可以直接删掉下面4行）
            Sensor_LegsPos->Par_HipLength   = Sensor_LegsPos->KinematicParams.block<1,3>(0,3).norm();
            Sensor_LegsPos->Par_ThighLength = Sensor_LegsPos->KinematicParams.block<1,3>(0,6).norm();
            Sensor_LegsPos->Par_CalfLength  = Sensor_LegsPos->KinematicParams.block<1,3>(0,9).norm();
            Sensor_LegsPos->Par_FootLength  = std::abs(Sensor_LegsPos->KinematicParams(0,12));
            RCLCPP_INFO(get_node()->get_logger(), "Kinematic Params Sucessfully Read.");
        }
    }

    rclcpp::Parameter imu_acc_params;
    if (get_node()->get_parameter("imu_acc_params", imu_acc_params)) {
        auto v = imu_acc_params.as_double_array();
        if (v.size() == 6u) {
            Sensor_IMUAcc->SensorPosition[0]     = v[0];
            Sensor_IMUAcc->SensorPosition[1]     = v[1];
            Sensor_IMUAcc->SensorPosition[2]     = v[2];
            double r = v[3] * M_PI / 180.0;
            double p = v[4] * M_PI / 180.0;
            double y = v[5] * M_PI / 180.0;
            Eigen::AngleAxisd rollAngle (r, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle  (y, Eigen::Vector3d::UnitZ());
            Sensor_IMUAcc->SensorQuaternion = yawAngle * pitchAngle * rollAngle;
            Sensor_IMUAcc->SensorQuaternionInv = Sensor_IMUAcc->SensorQuaternion.inverse();
            RCLCPP_INFO(get_node()->get_logger(), "IMU Acc Params Sucessfully Read.");
        }
    }

    rclcpp::Parameter imu_gyro_params;
    if (get_node()->get_parameter("imu_gyro_params", imu_gyro_params)) {
        auto v = imu_gyro_params.as_double_array();
        if (v.size() == 6u) {
            Sensor_IMUMagGyro->SensorPosition[0] = v[0];
            Sensor_IMUMagGyro->SensorPosition[1] = v[1];
            Sensor_IMUMagGyro->SensorPosition[2] = v[2];
            double r = v[3] * M_PI / 180.0;
            double p = v[4] * M_PI / 180.0;
            double y = v[5] * M_PI / 180.0;
            Eigen::AngleAxisd rollAngle (r, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle  (y, Eigen::Vector3d::UnitZ());
            Sensor_IMUMagGyro->SensorQuaternion = yawAngle * pitchAngle * rollAngle;
            Sensor_IMUMagGyro->SensorQuaternionInv = Sensor_IMUMagGyro->SensorQuaternion.inverse();
            RCLCPP_INFO(get_node()->get_logger(), "IMU Gyro Params Sucessfully Read.");
        }
    }
    
    if (publish_odom_enable){
        FETest_publisher = get_node()->create_publisher<fusion_estimator::msg::FusionEstimatorTest>(pub_estimation_topic, 10);
        SMXFE_publisher   = get_node()->create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic, 10);
        SMXFE_2D_publisher= get_node()->create_publisher<nav_msgs::msg::Odometry>(pub_odom_2d_topic, 10);
        RCLCPP_INFO(get_node()->get_logger(), "Odometry Publishing Enabled.");
    } else {
        RCLCPP_INFO(get_node()->get_logger(), "Odometry Publishing Disabled.");
    }

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn legged_estimator::on_activate(const rclcpp_lifecycle::State &previous_state) 
{
    (void)previous_state;
    memset(LastIMU, 0, sizeof(LastIMU)); //初始化
    memset(LastJoint, 0, sizeof(LastJoint));
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> legged_estimator::on_export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(12+1);

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
        "legged_estimator", 
        "position_x",
        &fusion_msg.estimated_xyz[0])
        );

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
        "legged_estimator", 
        "position_y",
        &fusion_msg.estimated_xyz[3])
        );

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
        "legged_estimator", 
        "position_z",
        &fusion_msg.estimated_xyz[6])
        );
    //基座位置

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
        "legged_estimator", 
        "orientation_roll",
        &fusion_msg.estimated_rpy[0])
        );

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
        "legged_estimator", 
        "orientation_pitch",
        &fusion_msg.estimated_rpy[3])
        );

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
        "legged_estimator", 
        "orientation_yaw",
        &fusion_msg.estimated_rpy[6])
        );
    //基座欧拉角

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
        "legged_estimator", 
        "linear_x",
        &fusion_msg.estimated_xyz[1])
        );

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
        "legged_estimator", 
        "linear_y",
        &fusion_msg.estimated_xyz[4])
        );

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
        "legged_estimator", 
        "linear_z",
        &fusion_msg.estimated_xyz[7])
        );
    //基座线速度

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
        "legged_estimator", 
        "angular_x",
        &fusion_msg.estimated_rpy[1])
        );

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
        "legged_estimator", 
        "angular_y",
        &fusion_msg.estimated_rpy[4])
        );

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
        "legged_estimator", 
        "angular_z",
        &fusion_msg.estimated_rpy[7])
        );
    //基座角速度

    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
        "legged_estimator", 
        "contact_state",
        &contact_state_)
        );

    return state_interfaces;
}

controller_interface::InterfaceConfiguration legged_estimator::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(10 + joint_names_.size()*2 + tipforce_names_.size());

    config.names.push_back(imu_name_ + "/orientation_x");
    config.names.push_back(imu_name_ + "/orientation_y");
    config.names.push_back(imu_name_ + "/orientation_z");
    config.names.push_back(imu_name_ + "/orientation_w");

    config.names.push_back(imu_name_ + "/linear_acceleration_x");
    config.names.push_back(imu_name_ + "/linear_acceleration_y");
    config.names.push_back(imu_name_ + "/linear_acceleration_z");

    config.names.push_back(imu_name_ + "/angular_velocity_x");
    config.names.push_back(imu_name_ + "/angular_velocity_y");
    config.names.push_back(imu_name_ + "/angular_velocity_z");
    //申请IMU消息接口

    for (const auto &joint_name : joint_names_){
        config.names.push_back(joint_name + "/position");
        
    }
    //申请关节位置接口
    for (const auto &joint_name : joint_names_){
        config.names.push_back(joint_name + "/velocity");
    }
    //申请关节速度接口
    
    for (const auto &tipforce_name : tipforce_names_){
        config.names.push_back(tipforce_name + "/effort");
    }
    //申请足端力接口
    
    return config;
}

controller_interface::InterfaceConfiguration legged_estimator::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
}

controller_interface::return_type legged_estimator::update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void) time;
    (void) period;
    return controller_interface::return_type::OK;
}

controller_interface::return_type legged_estimator::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void) period;
    fusion_msg.stamp = time;
    CurrentTimestamp = time.seconds();

    uint8_t i = 0;
    for (i = 0; i<10; i++){
        // const auto imu_opt = state_interfaces_[i].get_optional<double>(10);
        // if (!imu_opt){
        //     RCLCPP_WARN(get_node()->get_logger(), "legged_estimator: IMU data timeout.");
        //     return controller_interface::return_type::ERROR;
        // }
        imu_msg_.data[i] = state_interfaces_[i].get_optional().value();
    }
    //获取IMU消息

    for (i = 0; i < 12; i++){
        // const auto jointpos_opt = state_interfaces_[i+10].get_optional<double>(10);
        // if (!jointpos_opt){
        //     RCLCPP_WARN(get_node()->get_logger(), "legged_estimator: Joint %s postion data timeout.", joint_names_[i].c_str());
        //     return controller_interface::return_type::ERROR;
        // }
        joint_msg_[i] = state_interfaces_[i+10].get_optional().value();
    }
    //获取关节位置消息

    for (i = 0; i < 12; i++){
        // const auto jointvel_opt = state_interfaces_[i+22].get_optional<double>(10);
        // if (!jointvel_opt){
        //     RCLCPP_WARN(get_node()->get_logger(), "legged_estimator: Joint %s velocity data timeout.", joint_names_[i].c_str());
        //     return controller_interface::return_type::ERROR;
        // }
        joint_msg_[12 + i] = state_interfaces_[i+22].get_optional().value();
    }
    //获取关节速度消息

    for (i = 0; i < 4; i++){
        // const auto tipforce_opt = state_interfaces_[i+34].get_optional<double>(10);
        // if (!tipforce_opt){
        //     RCLCPP_WARN(get_node()->get_logger(), "legged_estimator: Tip force %s data timeout.", tipforce_names_[i].c_str());
        //     return controller_interface::return_type::ERROR;
        // }
        joint_msg_[24 + i] = state_interfaces_[i+34].get_optional().value();
    }
    //获取足端力消息

    imu_update(imu_msg_);
    joint_update(joint_msg_);

    if (publish_odom_enable){
        publish_estimation();
    }

    fusion_msg.stamp;
    
    return controller_interface::return_type::OK;
}

void legged_estimator::imu_update(const imu_msg_t msg)
{
    double LatestMessage[3][100]={0};
    double roll, pitch, yaw;

    tf2::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    LatestMessage[0][3*0+2] = msg.linear_acceleration.x;
    LatestMessage[0][3*1+2] = msg.linear_acceleration.y;
    LatestMessage[0][3*2+2] = msg.linear_acceleration.z;

    LatestMessage[1][3*0] = roll + orientation_correct[0];
    LatestMessage[1][3*1] = pitch + orientation_correct[3];
    LatestMessage[1][3*2] = yaw  + orientation_correct[6];
    LatestMessage[1][3*0+1] = msg.angular_velocity.x;
    LatestMessage[1][3*1+1] = msg.angular_velocity.y;
    LatestMessage[1][3*2+1] = msg.angular_velocity.z;
    
    for(int i = 0; i < 9; i++)
    {
        if(LastIMU[0][i] != LatestMessage[0][i])
            break;
        if(LastIMU[1][i] != LatestMessage[1][i])
            break;
        if(i==8)
            return;  //数据未变则视为重复发送，舍弃数据
    }
    for(int j = 0; j < 9; j++)
    {
        LastIMU[0][j] = LatestMessage[0][j];
        LastIMU[1][j] = LatestMessage[1][j];
    }

    Sensor_IMUAcc->SensorDataHandle(LatestMessage[0], CurrentTimestamp);
    Sensor_IMUMagGyro->SensorDataHandle(LatestMessage[1], CurrentTimestamp);

    for(int i=0; i<9; i++){
        fusion_msg.estimated_xyz[i] = StateSpaceModel_Go2_Sensors[0]->EstimatedState[i] + position_correct[i];
        fusion_msg.estimated_rpy[i] = StateSpaceModel_Go2_Sensors[1]->EstimatedState[i];
    }
}

void legged_estimator::joint_update(std::vector<double> arr)
{
    double LatestMessage[3][100]={0};

    if (arr.size() < 28) return;
    /* 数据布置：
    data[ 0..11] : 12× 关节位置  (q)
    data[12..23] : 12× 关节速度  (dq)
    data[24..27] : 4 × 足端力    (foot_force)
    */

    for(int LegNumber = 0; LegNumber<4; LegNumber++)
    {
        for(int i = 0; i < 3; i++)
        {
            LatestMessage[2][LegNumber*3+i] = arr[LegNumber*3+i];
            LatestMessage[2][12+LegNumber*3+i] = arr[12+LegNumber*3+i];
        }
        LatestMessage[2][24 + LegNumber] = arr[24 + LegNumber];
    }

    for(int i = 0; i < 28; i++)
    {
        if(LastJoint[2][i] != LatestMessage[2][i])
            break;
        if(i==27)
            return;  //数据未变则视为重复发送，舍弃数据
    }
    for(int j = 0; j < 28; j++)
    {
        LastJoint[2][j] = LatestMessage[2][j];
    }

    if(leg_pos_enable)
    {
        Sensor_LegsPos->SensorDataHandle(LatestMessage[2], CurrentTimestamp);
        
        for(int i=0; i<9; i++){
            fusion_msg.estimated_xyz[i] = StateSpaceModel_Go2_Sensors[0]->EstimatedState[i] + position_correct[i];
        }

        for(int i=0; i<4; i++){
            for(int j=0; j<3; j++){
                fusion_msg.feet_based_position[3 * i + j] = StateSpaceModel_Go2_Sensors[0]->Double_Par[48 + 6 * i + j];
                fusion_msg.feet_based_velocity[3 * i + j] = StateSpaceModel_Go2_Sensors[0]->Double_Par[48 + 6 * i + j + 3];
            }
        }
        
        for(int LegNumber=0; LegNumber<4; LegNumber++){
            for(int i=0; i<3; i++){
                fusion_msg.data_check_a[3 * LegNumber + i] = StateSpaceModel_Go2_Sensors[0]->Double_Par[6 * LegNumber + i];      //身体坐标系的足身相对位置
                fusion_msg.data_check_b[3 * LegNumber + i] = StateSpaceModel_Go2_Sensors[0]->Double_Par[24 + 6 * LegNumber + i]; //世界坐标系的足身相对位置
            }
        }
    }
    if(leg_ori_enable)
    {
        double Last_Yaw = StateSpaceModel_Go2_Sensors[1]->EstimatedState[6] - orientation_correct[6];

        StateSpaceModel_Go2_Sensors[1]->Double_Par[97] = leg_ori_init_weight;
        StateSpaceModel_Go2_Sensors[1]->Double_Par[98] = leg_ori_time_wight;
        Sensor_LegsOri->SensorDataHandle(LatestMessage[2], CurrentTimestamp);

        orientation_correct[6] = StateSpaceModel_Go2_Sensors[1]->Double_Par[99] - Last_Yaw;
        if(!imu_data_enable)
            StateSpaceModel_Go2_Sensors[1]->EstimatedState[6] = StateSpaceModel_Go2_Sensors[1]->Double_Par[99];

        for(int i=0; i<9; i++){
            fusion_msg.estimated_rpy[i] = StateSpaceModel_Go2_Sensors[1]->EstimatedState[i];
        }
    }
}

void legged_estimator::publish_estimation()
{
    FETest_publisher->publish(fusion_msg);

    // 构造标准 3D odometry 消息，并发布
    nav_msgs::msg::Odometry SMXFE_odom;
    SMXFE_odom.header.stamp = fusion_msg.stamp;
    SMXFE_odom.header.frame_id = odom_frame_id;
    SMXFE_odom.child_frame_id = child_frame_id;

    // 使用 fusion_msg.estimated_xyz 的前 3 个元素作为位置
    SMXFE_odom.pose.pose.position.x = fusion_msg.estimated_xyz[0];
    SMXFE_odom.pose.pose.position.y = fusion_msg.estimated_xyz[3];
    SMXFE_odom.pose.pose.position.z = fusion_msg.estimated_xyz[6];

    // 使用 fusion_msg.estimated_rpy 的前 3 个元素（roll, pitch, yaw）转换为四元数
    tf2::Quaternion q;
    q.setRPY(fusion_msg.estimated_rpy[0], fusion_msg.estimated_rpy[3], fusion_msg.estimated_rpy[6]);
    SMXFE_odom.pose.pose.orientation = tf2::toMsg(q);

    // 线速度：使用 fusion_msg.estimated_xyz 的索引 1, 4, 7
    SMXFE_odom.twist.twist.linear.x = fusion_msg.estimated_xyz[1];
    SMXFE_odom.twist.twist.linear.y = fusion_msg.estimated_xyz[4];
    SMXFE_odom.twist.twist.linear.z = fusion_msg.estimated_xyz[7];

    // 角速度：使用 fusion_msg.estimated_rpy 的索引 1, 4, 7
    SMXFE_odom.twist.twist.angular.x = fusion_msg.estimated_rpy[1];
    SMXFE_odom.twist.twist.angular.y = fusion_msg.estimated_rpy[4];
    SMXFE_odom.twist.twist.angular.z = fusion_msg.estimated_rpy[7];

    // 设置位姿（pose）协方差：6x6 矩阵（行优先排列）
    // 初始化全部置零
    for (int i = 0; i < 36; ++i) {
        SMXFE_odom.pose.covariance[i] = 0.0;
    }
    // 位置 (x, y, z) 的协方差设为 0.01
    SMXFE_odom.pose.covariance[0]  = 0.1;   // x
    SMXFE_odom.pose.covariance[7]  = 0.1;   // y
    SMXFE_odom.pose.covariance[14] = 0.1;   // z
    // 姿态（roll, pitch, yaw）的协方差设为 0.0001
    SMXFE_odom.pose.covariance[21] = 0.1; // roll
    SMXFE_odom.pose.covariance[28] = 0.1; // pitch
    SMXFE_odom.pose.covariance[35] = 0.1; // yaw

    // 设置 twist 协方差：6x6 矩阵（行优先排列）
    for (int i = 0; i < 36; ++i) {
        SMXFE_odom.twist.covariance[i] = 0.0;
    }
    // 线速度 (x, y, z) 的协方差设为 0.1
    SMXFE_odom.twist.covariance[0]  = 0.1;   // linear x
    SMXFE_odom.twist.covariance[7]  = 0.1;   // linear y
    SMXFE_odom.twist.covariance[14] = 0.1;   // linear z
    // 角速度 (x, y, z) 的协方差设为 0.01
    SMXFE_odom.twist.covariance[21] = 0.1;  // angular x
    SMXFE_odom.twist.covariance[28] = 0.1;  // angular y
    SMXFE_odom.twist.covariance[35] = 0.1;  // angular z

    // 发布 odometry 消息
    SMXFE_publisher->publish(SMXFE_odom);

    // 构造标准 2D odometry 消息，并发布
    nav_msgs::msg::Odometry SMXFE_odom_2D;
    SMXFE_odom_2D.header.stamp = fusion_msg.stamp;
    SMXFE_odom_2D.header.frame_id = odom_frame_id;
    SMXFE_odom_2D.child_frame_id = child_frame_2d_id;

    // 使用 fusion_msg.estimated_xyz 的前 3 个元素作为位置
    SMXFE_odom_2D.pose.pose.position.x = fusion_msg.estimated_xyz[0];
    SMXFE_odom_2D.pose.pose.position.y = fusion_msg.estimated_xyz[3];
    SMXFE_odom_2D.pose.pose.position.z = 0;

    // 使用 fusion_msg.estimated_rpy 的前 3 个元素（roll, pitch, yaw）转换为四元数
    q.setRPY(0, 0, fusion_msg.estimated_rpy[6]);
    SMXFE_odom_2D.pose.pose.orientation = tf2::toMsg(q);

    // 线速度：使用 fusion_msg.estimated_xyz 的索引 1, 4, 7
    SMXFE_odom_2D.twist.twist.linear.x = fusion_msg.estimated_xyz[1];
    SMXFE_odom_2D.twist.twist.linear.y = fusion_msg.estimated_xyz[4];
    SMXFE_odom_2D.twist.twist.linear.z = fusion_msg.estimated_xyz[7];

    // 角速度：使用 fusion_msg.estimated_rpy 的索引 1, 4, 7
    SMXFE_odom_2D.twist.twist.angular.x = fusion_msg.estimated_rpy[1];
    SMXFE_odom_2D.twist.twist.angular.y = fusion_msg.estimated_rpy[4];
    SMXFE_odom_2D.twist.twist.angular.z = fusion_msg.estimated_rpy[7];

    // 设置位姿（pose）协方差：6x6 矩阵（行优先排列）
    // 初始化全部置零
    for (int i = 0; i < 36; ++i) {
        SMXFE_odom_2D.pose.covariance[i] = 0.0;
    }
    // 位置 (x, y, z) 的协方差设为 0.01
    SMXFE_odom_2D.pose.covariance[0]  = 0.1;   // x
    SMXFE_odom_2D.pose.covariance[7]  = 0.1;   // y
    SMXFE_odom_2D.pose.covariance[14] = 0.1;   // z
    // 姿态（roll, pitch, yaw）的协方差设为 0.0001
    SMXFE_odom_2D.pose.covariance[21] = 0.1; // roll
    SMXFE_odom_2D.pose.covariance[28] = 0.1; // pitch
    SMXFE_odom_2D.pose.covariance[35] = 0.1; // yaw

    // 设置 twist 协方差：6x6 矩阵（行优先排列）
    for (int i = 0; i < 36; ++i) {
        SMXFE_odom_2D.twist.covariance[i] = 0.0;
    }
    // 线速度 (x, y, z) 的协方差设为 0.1
    SMXFE_odom_2D.twist.covariance[0]  = 0.1;   // linear x
    SMXFE_odom_2D.twist.covariance[7]  = 0.1;   // linear y
    SMXFE_odom_2D.twist.covariance[14] = 0.1;   // linear z
    // 角速度 (x, y, z) 的协方差设为 0.01
    SMXFE_odom_2D.twist.covariance[21] = 0.1;  // angular x
    SMXFE_odom_2D.twist.covariance[28] = 0.1;  // angular y
    SMXFE_odom_2D.twist.covariance[35] = 0.1;  // angular z

    SMXFE_2D_publisher->publish(SMXFE_odom_2D);
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(legged_robot::legged_estimator, controller_interface::ChainableControllerInterface)