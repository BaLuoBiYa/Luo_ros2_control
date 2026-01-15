#include "legged_controllers/legged_controller.hpp"

namespace legged
{
    controller_interface::CallbackReturn legged_controller::on_init()
    {
        // Initialize parameters
        robotName = auto_declare<std::string>("robotName", "legged_robot");
        taskFile = auto_declare<std::string>("taskFile", "");
        urdfFile = auto_declare<std::string>("urdfFile", "");
        referenceFile = auto_declare<std::string>("referenceFile", "");

        joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
        if (joint_names_.size() != 12u)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(), "Expected 12 joint names, got %zu", joint_names_.size());
            return CallbackReturn::ERROR;
        }

        imu_name_ = auto_declare<std::string>("imu_name", "");
        visualize_ = auto_declare<bool>("visualize", true);
        ocs2::loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose_);

        imuSensorHandle_ = std::make_shared<semantic_components::IMUSensor>(imu_name_);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration legged_controller::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(12 + 12 + 12);
        for (const auto &joint_name : joint_names_)
        {
            config.names.push_back(joint_name + "/position");
        }
        // 申请关节位置命令接口
        for (const auto &joint_name : joint_names_)
        {
            config.names.push_back(joint_name + "/velocity");
        }
        // 申请关节速度命令接口
        for (const auto &joint_name : joint_names_)
        {
            config.names.push_back(joint_name + "/effort");
        }
        // 申请关节力命令接口
        return config;
    }

    controller_interface::InterfaceConfiguration legged_controller::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(12 + 12 + 12 + 4 + 10);

        for (const auto &joint_name : joint_names_)
        {
            config.names.push_back(joint_name + "/position");
        }
        // 申请关节位置状态接口
        for (const auto &joint_name : joint_names_)
        {
            config.names.push_back(joint_name + "/velocity");
        }
        // 申请关节速度状态接口

        config.names.push_back("LF/contact");
        config.names.push_back("LH/contact");
        config.names.push_back("RF/contact");
        config.names.push_back("RH/contact");
        // 申请足端接触力状态接口

        std::vector<std::string> imu_interfaces =
            imuSensorHandle_->get_state_interface_names();
        config.names.insert(config.names.end(), imu_interfaces.begin(), imu_interfaces.end());
        // 申请IMU状态接口

        return config;
    }

    controller_interface::CallbackReturn legged_controller::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;

        if (!imuSensorHandle_->assign_loaned_state_interfaces(state_interfaces_))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to assign IMU interfaces");
            return CallbackReturn::ERROR;
        }
        // 分配IMU状态接口

        leggedInterface_ =
            std::make_shared<ocs2::legged_robot::LeggedRobotInterface>(taskFile, urdfFile, referenceFile);
        // 创建机器人接口

        mrtInterface_ = std::make_shared<mrtInterface>(robotName, get_node()->shared_from_this());
        // 创建MRT接口

        CentroidalModelPinocchioMapping pinocchioMapping(
            leggedInterface_->getCentroidalModelInfo());

        eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
            leggedInterface_->getPinocchioInterface(),
            pinocchioMapping,
            leggedInterface_->modelSettings().contactNames3DoF);

        robotVisualizer_ = std::make_shared<ocs2::legged_robot::LeggedRobotVisualizer>(
            leggedInterface_->getPinocchioInterface(),
            leggedInterface_->getCentroidalModelInfo(),
            *eeKinematicsPtr_,
            get_node()->shared_from_this());
        // visualization

        wbc_ = std::make_shared<WeightedWbc>(
            leggedInterface_->getPinocchioInterface(),
            leggedInterface_->getCentroidalModelInfo(),
            *eeKinematicsPtr_);
        wbc_->loadTasksSetting(taskFile, verbose_);
        // Whole body control

        stateEstimate_ = std::make_shared<KalmanFilterEstimate>(
            leggedInterface_->getPinocchioInterface(),
            leggedInterface_->getCentroidalModelInfo(),
            *eeKinematicsPtr_,
            get_node()->shared_from_this());
        dynamic_cast<KalmanFilterEstimate &>(*stateEstimate_).loadSettings(taskFile, verbose_);
        currentObservation_.time = 0;
        // State Estimation

        safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());
        // Safety Checker

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn legged_controller::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        mrtInterface_->initRollout(&leggedInterface_->getRollout());
        mrtInterface_->launchNodes();

        // Initial state
        ocs2::SystemObservation initObservation;
        initObservation.state = leggedInterface_->getInitialState();
        initObservation.input =
            ocs2::vector_t::Zero(leggedInterface_->getCentroidalModelInfo().inputDim);
        initObservation.mode = ocs2::legged_robot::ModeNumber::STANCE;

        // Initial command
        ocs2::TargetTrajectories initTargetTrajectories(
            {0.0},
            {initObservation.state},
            {initObservation.input});
        RCLCPP_INFO_STREAM(get_node()->get_logger(), "Waiting for the initial policy ...");

        // Reset MPC node
        mrtInterface_->resetMpcNode(initTargetTrajectories);

        // Wait for the initial policy
        while (!mrtInterface_->initialPolicyReceived() && rclcpp::ok())
        {
            rclcpp::sleep_for(std::chrono::milliseconds(10));
            mrtInterface_->setCurrentObservation(initObservation);
        }
        RCLCPP_INFO_STREAM(get_node()->get_logger(), "Initial policy has been received.");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type legged_controller::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        updateEstimation(time, period);
        // Update the current state of the system

        mrtInterface_->setCurrentObservation(currentObservation_);
        mrtInterface_->updatePolicy();
        // Load the latest MPC policy
        
        vector_t optimizedState, optimizedInput;
        size_t plannedMode = 0; // The mode that is active at the time the policy is evaluated at.
        mrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);
        // Evaluate the current policy

        if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput))
        {
            RCLCPP_ERROR_STREAM(get_node()->get_logger(), "[Legged Controller] Safety check failed, stopping the controller.");
            return controller_interface::return_type::ERROR;
        }
        // Safety check, if failed, stop the controller

        currentObservation_.input = optimizedInput;
        vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.seconds());
        // Whole body control

        vector_t torque = x.tail(12);
        vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
        vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());
        // Extract commands

        bool writeSuccess = true;
        for (size_t j = 0; j < 12; ++j)
        {
            writeSuccess = command_interfaces_[j].set_value<double>(posDes(j));
            writeSuccess =command_interfaces_[j + 12].set_value<double>(velDes(j));
            writeSuccess = command_interfaces_[j + 24].set_value<double>(torque(j));
        }
        // Send commands to the robot
        if (!writeSuccess)
        {
            RCLCPP_ERROR_STREAM(get_node()->get_logger(), "[Legged Controller] Command write failed, stopping the controller.");
            return controller_interface::return_type::ERROR;
        }

        if (visualize_)
        {
            robotVisualizer_->update(currentObservation_, mrtInterface_->getPolicy(), mrtInterface_->getCommand());
        }
        // Visualization

        return controller_interface::return_type::OK;
    }

    // controller_interface::CallbackReturn legged_controller::on_error(const rclcpp_lifecycle::State &previous_state)
    // {
    //     bool writeSuccess = true;
    //     for (size_t j = 0; j < 12; ++j)
    //     {
    //         writeSuccess = command_interfaces_[j].set_value(0);
    //         writeSuccess =command_interfaces_[j + 12].set_value(0);
    //         writeSuccess = command_interfaces_[j + 24].set_value(0);
    //     }
    //     return controller_interface::CallbackReturn::SUCCESS;
    // }


    void legged_controller::updateEstimation(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        ocs2::vector_t jointPos(12), jointVel(12);
        // ocs2::legged_robot::contact_flag_t contacts;
        Eigen::Quaternion<scalar_t> quat;
        ocs2::legged_robot::contact_flag_t contactFlag;
        ocs2::legged_robot::vector3_t angularVel, linearAccel;
        // ocs2::legged_robot::matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

        for (size_t i = 0; i < 12; ++i)
        {
            auto pos = state_interfaces_[i].get_optional<double>();
            if (pos == std::nullopt)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to read joint position interface");
                continue ;
            }
            jointPos(i) = pos.value();
        }

        for (size_t i = 0; i < 12; ++i)
        {
            auto vel = state_interfaces_[12 + i].get_optional<double>();
            if (vel == std::nullopt)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to read joint velocity interface");
                continue ;
            }
            jointVel(i) = vel.value();
        }

        for (size_t i = 0; i < 4; ++i)
        {
            auto contact = state_interfaces_[24 + i].get_optional<bool>();
            if (contact == std::nullopt)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to read contact interface");
                continue ;
            }
            contactFlag[i] = contact.value();
        }
        // 读取关节状态和接触状态,如果读取失败，跳过更新

        for (size_t i = 0; i < 4; ++i)
        {
            quat.coeffs()(i) = imuSensorHandle_->get_orientation()[i];
        }
        for (size_t i = 0; i < 3; ++i)
        {
            angularVel(i) = imuSensorHandle_->get_angular_velocity()[i];
            linearAccel(i) = imuSensorHandle_->get_linear_acceleration()[i];
        }
        // for (size_t i = 0; i < 9; ++i)
        // {
        //     orientationCovariance(i) = imuSensorHandle_->get_orientation_covariance()[i];
        //     angularVelCovariance(i) = imuSensorHandle_->get_angular_velocity_covariance()[i];
        //     linearAccelCovariance(i) = imuSensorHandle_->get_linear_acceleration_covariance()[i];
        // }

        stateEstimate_->updateJointStates(jointPos, jointVel);
        stateEstimate_->updateContact(contactFlag);
        stateEstimate_->updateImu(
            quat,
            angularVel,
            linearAccel);

        measuredRbdState_ = stateEstimate_->update(time, period);

        currentObservation_.time += period.seconds();
        scalar_t yawLast = currentObservation_.state(9);
        currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
        currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
        currentObservation_.mode = stateEstimate_->getMode();
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(legged::legged_controller, controller_interface::ControllerInterface)