#include "legged_controllers/legged_controller.hpp"

namespace legged {
    controller_interface::CallbackReturn legged_controller::on_init() {
        // Initialize parameters
        robotName = auto_declare<std::string>("robotName", "legged_robot");
        taskFile = auto_declare<std::string>("taskFile", "");
        urdfFile = auto_declare<std::string>("urdfFile", "");
        referenceFile = auto_declare<std::string>("referenceFile", "");

        jointNames_ = auto_declare<std::vector<std::string>>("joints", {});
        if (jointNames_.size() != 12u) {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected 12 joint names, got %zu", jointNames_.size());
            return CallbackReturn::ERROR;
        }

        imuName_ = auto_declare<std::string>("imuName", "imu");
        visualize_ = auto_declare<bool>("visualize", true);
        ocs2::loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose_);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration legged_controller::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::NONE;

        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(12 + 12 + 12);
        for (const auto &joint_name : jointNames_) {
            config.names.push_back(joint_name + "/position");
        }
        // 申请关节位置命令接口
        for (const auto &joint_name : jointNames_) {
            config.names.push_back(joint_name + "/velocity");
        }
        // 申请关节速度命令接口
        for (const auto &joint_name : jointNames_) {
            config.names.push_back(joint_name + "/effort");
        }
        // 申请关节力命令接口
        return config;
    }

    controller_interface::InterfaceConfiguration legged_controller::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::NONE;

        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(12 + 12 + 4 + 10);

        for (const std::string &joint_name : jointNames_) {
            config.names.push_back(joint_name + "/position");
        }
        // 申请关节位置状态接口
        for (const std::string &joint_name : jointNames_) {
            config.names.push_back(joint_name + "/velocity");
        }
        // 申请关节速度状态接口

        config.names.push_back("contact/LF");
        config.names.push_back("contact/LH");
        config.names.push_back("contact/RF");
        config.names.push_back("contact/RH");
        // 申请足端接触力状态接口

        config.names.push_back(imuName_ + "/orientation.x");
        config.names.push_back(imuName_ + "/orientation.y");
        config.names.push_back(imuName_ + "/orientation.z");
        config.names.push_back(imuName_ + "/orientation.w");

        config.names.push_back(imuName_ + "/angular_velocity.x");
        config.names.push_back(imuName_ + "/angular_velocity.y");
        config.names.push_back(imuName_ + "/angular_velocity.z");

        config.names.push_back(imuName_ + "/linear_acceleration.x");
        config.names.push_back(imuName_ + "/linear_acceleration.y");
        config.names.push_back(imuName_ + "/linear_acceleration.z");
        // 申请IMU状态接口

        return config;
    }

    controller_interface::CallbackReturn
    legged_controller::on_configure(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;

        leggedInterface_ =
            std::make_shared<ocs2::legged_robot::LeggedRobotInterface>(taskFile, urdfFile, referenceFile);
        // 创建机器人接口

        mrtInterface_ = std::make_shared<mrtInterface>(robotName, get_node()->shared_from_this());
        // 创建MRT接口

        CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());

        eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
            leggedInterface_->getPinocchioInterface(), pinocchioMapping,
            leggedInterface_->modelSettings().contactNames3DoF);

        robotVisualizer_ = std::make_shared<ocs2::legged_robot::LeggedRobotVisualizer>(
            leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_,
            get_node()->shared_from_this());
        // visualization

        wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(),
                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
        wbc_->loadTasksSetting(taskFile, verbose_);
        // Whole body control

        stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                                leggedInterface_->getCentroidalModelInfo(),
                                                                *eeKinematicsPtr_, get_node()->shared_from_this());
        dynamic_cast<KalmanFilterEstimate &>(*stateEstimate_).loadSettings(taskFile, verbose_);
        currentObservation_.time = 0;
        rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                          leggedInterface_->getCentroidalModelInfo());
        measuredRbdState_.setZero(2 * leggedInterface_->getCentroidalModelInfo().generalizedCoordinatesNum);
        // State Estimation

        safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());
        // Safety Checker

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn legged_controller::on_activate(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;

        mrtInterface_->initRollout(&leggedInterface_->getRollout());
        mrtInterface_->launchNodes();

        // Initial state
        currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
        currentObservation_.state = leggedInterface_->getInitialState();
        currentObservation_.time = get_node()->now().seconds();
        currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
        currentObservation_.mode = ocs2::legged_robot::ModeNumber::STANCE;

        // Initial command
        ocs2::TargetTrajectories initTargetTrajectories({0.0}, {currentObservation_.state},
                                                        {currentObservation_.input});

        // Set the first observation and command and wait for optimization to finish
        mrtInterface_->setCurrentObservation(currentObservation_);

        // Reset MPC node
        mrtInterface_->resetMpcNode(initTargetTrajectories);
        // Wait for the initial policy
        while (!mrtInterface_->initialPolicyReceived() && rclcpp::ok()) {
            mrtInterface_->setCurrentObservation(currentObservation_);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Waiting for the initial policy ...");
        }
        RCLCPP_INFO_STREAM(get_node()->get_logger(), "Initial policy has been received.");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type legged_controller::update(const rclcpp::Time &time,
                                                                const rclcpp::Duration &period) {
        this->updateEstimation(time, period);
        // Update the current state of the system

        mrtInterface_->setCurrentObservation(currentObservation_);
        mrtInterface_->updatePolicy();
        // Load the latest MPC policy

        vector_t optimizedState, optimizedInput;
        size_t plannedMode = 0; // The mode that is active at the time the policy is evaluated at.
        mrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState,
                                      optimizedInput, plannedMode);
        // Evaluate the current policy

        if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
            RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                                "[Legged Controller] Safety check failed, stopping the controller.");
            return controller_interface::return_type::ERROR;
        }
        // Safety check, if failed, stop the controller

        currentObservation_.input = optimizedInput;
        vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.seconds());
        // Whole body control

        vector_t torque = x.tail(12);
        vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
        vector_t velDes =
            centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());
        // Extract commands

        bool writeSuccess = true;
        for (size_t j = 0; j < 12; ++j) {
            writeSuccess &= command_interfaces_[j].set_value<double>(0);
            writeSuccess &= command_interfaces_[j + 12].set_value<double>(0);
            writeSuccess &= command_interfaces_[j + 24].set_value<double>(0);
        }
        // Send commands to the robot
        if (!writeSuccess) {
            RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                                "[Legged Controller] Command write failed, stopping the controller.");
            return controller_interface::return_type::ERROR;
        }

        if (visualize_) {
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

    void legged_controller::updateEstimation(const rclcpp::Time &time, const rclcpp::Duration &period) {
        ocs2::vector_t jointPos(12), jointVel(12);
        // ocs2::legged_robot::contact_flag_t contacts;
        Eigen::Quaternion<scalar_t> quat;
        ocs2::legged_robot::contact_flag_t contactFlag;
        ocs2::legged_robot::vector3_t angularVel, linearAccel;
        // ocs2::legged_robot::matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

        for (size_t i = 0; i < 12; ++i) {
            auto pos = state_interfaces_[i].get_optional<double>();
            if (pos == std::nullopt) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to read joint position interface");
                continue;
            }
            jointPos(i) = pos.value();
        }

        for (size_t i = 0; i < 12; ++i) {
            auto vel = state_interfaces_[12 + i].get_optional<double>();
            if (vel == std::nullopt) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to read joint velocity interface");
                continue;
            }
            jointVel(i) = vel.value();
        }

        for (size_t i = 0; i < 4; ++i) {
            auto contact = state_interfaces_[24 + i].get_optional<bool>();
            if (contact == std::nullopt) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to read contact interface");
                continue;
            }
            contactFlag[i] = contact.value();
        }
        // 读取关节状态和接触状态,如果读取失败，跳过更新

        for (size_t i = 0; i < 4; ++i) {
            auto imu_orient = state_interfaces_[28 + i].get_optional<double>();
            if (imu_orient == std::nullopt) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to read IMU orientation interface");
                continue;
            }
            quat.coeffs()(i) = imu_orient.value();
        }

        for (size_t i = 0; i < 3; ++i) {
            auto imu_angular_vel = state_interfaces_[32 + i].get_optional<double>();
            if (imu_angular_vel == std::nullopt) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to read IMU angular velocity interface");
                continue;
            }
            angularVel(i) = imu_angular_vel.value();
            auto imu_linear_accel = state_interfaces_[35 + i].get_optional<double>();
            if (imu_linear_accel == std::nullopt) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to read IMU linear acceleration interface");
                continue;
            }
            linearAccel(i) = imu_linear_accel.value();
        }

        stateEstimate_->updateJointStates(jointPos, jointVel);
        stateEstimate_->updateContact(contactFlag);
        stateEstimate_->updateImu(quat, angularVel, linearAccel);
        measuredRbdState_ = stateEstimate_->update(time, period);

        currentObservation_.time = time.seconds();
        scalar_t yawLast = currentObservation_.state(9);
        currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
        currentObservation_.state(9) =
            yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
        currentObservation_.mode = stateEstimate_->getMode();
    }
} // namespace legged

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(legged::legged_controller, controller_interface::ControllerInterface)