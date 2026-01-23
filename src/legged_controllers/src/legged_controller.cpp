#include "legged_controllers/legged_controller.hpp"

#include <pinocchio/multibody/model.hpp>

namespace legged {
    controller_interface::CallbackReturn legged_controller::on_init() {
        // Initialize parameters
        robotName_ = auto_declare<std::string>("robotName", "legged_robot");
        taskFile_ = auto_declare<std::string>("taskFile", "");
        urdfFile_ = auto_declare<std::string>("urdfFile", "");
        referenceFile_ = auto_declare<std::string>("referenceFile", "");

        jointNames_ = auto_declare<std::vector<std::string>>("joints", {});
        if (jointNames_.size() != 12u) {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected 12 joint names, got %zu", jointNames_.size());
            return CallbackReturn::ERROR;
        }

        imuName_ = auto_declare<std::string>("imuName", "imu");
        visualize_ = auto_declare<bool>("visualize", true);

        kp_ = auto_declare<double>("kp", 0.0);
        kd_ = auto_declare<double>("kd", 0.0);
        kf_ = auto_declare<double>("kf", 0.0);
        ocs2::loadData::loadCppDataType(taskFile_, "legged_robot_interface.verbose", verbose_);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration legged_controller::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::NONE;

        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(12);
        // for (const auto &joint_name : jointNames_) {
        //     config.names.push_back(joint_name + "/position");
        // }
        // // 申请关节位置命令接口
        // for (const auto &joint_name : jointNames_) {
        //     config.names.push_back(joint_name + "/velocity");
        // }
        // // 申请关节速度命令接口
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
        config.names.push_back("contact/RF");
        config.names.push_back("contact/LH");
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

        bool use_sim_time = false;
        if (!get_node()->has_parameter("use_sim_time")) {
            get_node()->declare_parameter("use_sim_time", false);
        }
        get_node()->get_parameter("use_sim_time", use_sim_time);
        rclcpp::NodeOptions node_opts;
        node_opts.context(get_node()->get_node_base_interface()->get_context());
        node_opts.start_parameter_services(false);
        node_opts.start_parameter_event_publisher(false);
        // 读取原节点参数

        leggedInterface_ = std::make_shared<LeggedInterface>(taskFile_, urdfFile_, referenceFile_);
        leggedInterface_->setupOptimalControlProblem(taskFile_, urdfFile_, referenceFile_, verbose_);
        // 创建机器人接口

        mpc_ = std::make_shared<ocs2::SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                              leggedInterface_->getOptimalControlProblem(),
                                              leggedInterface_->getInitializer());
        mpcNode_ = std::make_shared<rclcpp::Node>(robotName_ + "_mpc", get_node()->get_namespace(), node_opts);
        // 创建mpc节点

        std::shared_ptr<GaitSchedule> gaitSchedule =
            leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule();
        std::shared_ptr<ocs2::legged_robot::GaitReceiver> gaitReceiverPtr =
            std::make_shared<ocs2::legged_robot::GaitReceiver>(mpcNode_, gaitSchedule, robotName_);
        // Gait receiver

        std::shared_ptr<ocs2::RosReferenceManager> rosReferenceManagerPtr =
            std::make_shared<RosReferenceManager>(robotName_, leggedInterface_->getReferenceManagerPtr());
        rosReferenceManagerPtr->subscribe(mpcNode_);
        mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
        mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
        mpcObservationPublisher_ =
            mpcNode_->create_publisher<ocs2_msgs::msg::MpcObservation>(robotName_ + "_mpc_observation", 1);
        // ROS ReferenceManager

        mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
        mpcTimer_.reset();
        controllerTimer_.reset();
        controllerRunning_ = true;
        mpcRunning_ = false;
        mpcError_ = false;
        mpcThread_ = std::thread(&legged_controller::mpcTask, this);
        ocs2::setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
        // 创建MPC任务

        CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
        eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
            leggedInterface_->getPinocchioInterface(), pinocchioMapping,
            leggedInterface_->modelSettings().contactNames3DoF);

        if (visualize_) {
            visualizeNode_ =
                std::make_shared<rclcpp::Node>(robotName_ + "_viz", get_node()->get_namespace(), node_opts);
            visualizeNode_->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));
            robotVisualizer_ = std::make_shared<ocs2::legged_robot::LeggedRobotVisualizer>(
                leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                *eeKinematicsPtr_, visualizeNode_);
            // visualization
        }

        wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(),
                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
        wbc_->loadTasksSetting(taskFile_, verbose_);
        // Whole body control

        stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                                leggedInterface_->getCentroidalModelInfo(),
                                                                *eeKinematicsPtr_, get_node()->shared_from_this());
        dynamic_cast<legged::KalmanFilterEstimate &>(*stateEstimate_).loadSettings(taskFile_, verbose_);
        rbdConversions_ = std::make_shared<ocs2::CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                          leggedInterface_->getCentroidalModelInfo());
        measuredRbdState_.setZero(2 * leggedInterface_->getCentroidalModelInfo().generalizedCoordinatesNum);
        // State Estimation

        safetyChecker_ = std::make_shared<legged::SafetyChecker>(leggedInterface_->getCentroidalModelInfo());
        // Safety Checker

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn legged_controller::on_activate(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;

        currentObservation_.state = leggedInterface_->getInitialState();
        updateEstimation(get_node()->now(),
                         rclcpp::Duration(0, static_cast<uint32_t>((1.0 / get_update_rate()) * 1e9)));
        currentObservation_.input = vector_t::Zero(leggedInterface_->getCentroidalModelInfo().inputDim);
        currentObservation_.mode = ocs2::legged_robot::ModeNumber::STANCE;

        const double t0 = get_node()->now().seconds();
        ocs2::TargetTrajectories initTargetTrajectories({t0}, {currentObservation_.state}, {currentObservation_.input});
        // Initial state

        mpcMrtInterface_->reset();
        mpc_->reset();
        mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
        mpcMrtInterface_->setCurrentObservation(currentObservation_);
        mpcMrtInterface_->getReferenceManager().setTargetTrajectories(initTargetTrajectories);
        // Set the first observation and command and wait for optimization to finish

        RCLCPP_INFO_STREAM(get_node()->get_logger(), "Waiting for the initial policy ...");

        // Wait for the initial policy
        while (!mpcMrtInterface_->initialPolicyReceived() && rclcpp::ok()) {
            mpcMrtInterface_->advanceMpc();
            rclcpp::Rate(get_update_rate()).sleep();
        }
        RCLCPP_INFO_STREAM(get_node()->get_logger(), "Initial policy has been received.");
        mpcRunning_ = true;

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type legged_controller::update(const rclcpp::Time &time,
                                                                const rclcpp::Duration &period) {
        controllerTimer_.startTimer();

        if (mpcError_ == true) {
            return controller_interface::return_type::ERROR;
        }

        updateEstimation(time, period);
        mpcMrtInterface_->setCurrentObservation(currentObservation_);
        // Update the current state of the system

        mpcMrtInterface_->updatePolicy();
        // Load the latest MPC policy

        vector_t optimizedState, optimizedInput;
        size_t plannedMode = 0; // The mode that is active at the time the policy is evaluated at.
        mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState,
                                         optimizedInput, plannedMode);
        // Evaluate the current policy

        if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
            RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                                "[Legged Controller] Safety check failed, stopping the controller.");
            return controller_interface::return_type::ERROR;
        }
        // Safety check, if failed, stop the controller

        currentObservation_.input = optimizedInput;
        // vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.seconds());
        // Whole body control
        vector_t x;
        x.setZero(12);

        vector_t torque = x.tail(12);
        vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
        vector_t velDes =
            centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());
        // Extract commands

        double effort[12];
        for (size_t i = 0; i < 12; i++) {
            effort[i] = torque(i) * kf_ + (posDes(i) - jointPos_[i]) * kp_ + (velDes(i) - jointVel_[i]) * kd_;
        }

        bool writeSuccess = true;
        for (size_t j = 0; j < 12; ++j) {
            writeSuccess &= command_interfaces_[j].set_value<double>(effort[j]);
        }
        // Send commands to the robot
        if (!writeSuccess) {
            RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                                "[Legged Controller] Command write failed, stopping the controller.");
            return controller_interface::return_type::ERROR;
        }

        if (visualize_) {
            robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(),
                                     mpcMrtInterface_->getCommand());
        }
        // Visualization
        mpcObservationPublisher_->publish(ocs2::ros_msg_conversions::createObservationMsg(currentObservation_));

        controllerTimer_.endTimer();

        return controller_interface::return_type::OK;
    }

    void legged_controller::updateEstimation(const rclcpp::Time &time, const rclcpp::Duration &period) {

        for (size_t i = 0; i < 12; ++i) {
            auto pos = state_interfaces_[i].get_optional<double>();
            if (pos == std::nullopt) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to read joint position interface");
                continue;
            }
            jointPos_(i) = pos.value();
        }

        for (size_t i = 0; i < 12; ++i) {
            auto vel = state_interfaces_[12 + i].get_optional<double>();
            if (vel == std::nullopt) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to read joint velocity interface");
                continue;
            }
            jointVel_(i) = vel.value();
        }

        for (size_t i = 0; i < 4; ++i) {
            auto contact = state_interfaces_[24 + i].get_optional<bool>();
            if (contact == std::nullopt) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to read contact interface");
                continue;
            }
            contactFlag_[i] = contact.value();
        }
        // 读取关节状态和接触状态,如果读取失败，跳过更新

        for (size_t i = 0; i < 4; ++i) {
            auto imu_orient = state_interfaces_[28 + i].get_optional<double>();
            if (imu_orient == std::nullopt) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to read IMU orientation interface");
                continue;
            }
            quat_.coeffs()(i) = imu_orient.value();
        }

        for (size_t i = 0; i < 3; ++i) {
            auto imu_angular_vel = state_interfaces_[32 + i].get_optional<double>();
            if (imu_angular_vel == std::nullopt) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to read IMU angular velocity interface");
                continue;
            }
            angularVel_(i) = imu_angular_vel.value();
            auto imu_linear_accel = state_interfaces_[35 + i].get_optional<double>();
            if (imu_linear_accel == std::nullopt) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to read IMU linear acceleration interface");
                continue;
            }
            linearAccel_(i) = imu_linear_accel.value();
        }

        stateEstimate_->updateJointStates(jointPos_, jointVel_);
        stateEstimate_->updateContact(contactFlag_);
        stateEstimate_->updateImu(quat_, angularVel_, linearAccel_);
        measuredRbdState_ = stateEstimate_->update(time, period);

        currentObservation_.time = time.seconds();
        scalar_t yawLast = currentObservation_.state(9);
        currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
        currentObservation_.state(9) =
            yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
        currentObservation_.mode = stateEstimate_->getMode();
    }

    void legged_controller::mpcTask() {
        while (controllerRunning_) {
            try {
                ocs2::executeAndSleep(
                    [&]() {
                        if (mpcRunning_) {
                            mpcTimer_.startTimer();
                            mpcMrtInterface_->advanceMpc();
                            mpcTimer_.endTimer();
                        }
                    },
                    leggedInterface_->mpcSettings().mpcDesiredFrequency_);
            } catch (const std::exception &e) {
                controllerRunning_ = false;
                RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), "[Ocs2 MPC thread] Error : " << e.what());
                mpcError_ = true;
            }
        }
    }

    legged_controller::~legged_controller() {
        controllerRunning_ = false;
        if (mpcThread_.joinable()) {
            mpcThread_.join();
        }
        std::cerr << "########################################################################";
        std::cerr << "\n### MPC Benchmarking";
        std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
        std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
        std::cerr << "########################################################################";
        std::cerr << "\n### Controller Benchmarking";
        std::cerr << "\n###   Maximum : " << controllerTimer_.getMaxIntervalInMilliseconds() << "[ms].";
        std::cerr << "\n###   Average : " << controllerTimer_.getAverageInMilliseconds() << "[ms].";
    }

    controller_interface::CallbackReturn legged_controller::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        (void) previous_state;
        if (mpcThread_.joinable())
            mpcThread_.join();
        return CallbackReturn::SUCCESS;
    }

} // namespace legged

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(legged::legged_controller, controller_interface::ControllerInterface)