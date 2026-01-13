/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "legged_controllers/legged_mrt.hpp"

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

namespace legged
{
    MRT_ROS_Interface::MRT_ROS_Interface(std::string topicPrefix,const rclcpp_lifecycle::LifecycleNode::SharedPtr node)
        : topicPrefix_(topicPrefix),
          node_(node) {}

    MRT_ROS_Interface::~MRT_ROS_Interface() {}

    void MRT_ROS_Interface::resetMpcNode(const ocs2::TargetTrajectories &initTargetTrajectories)
    {
        this->reset();

        const auto resetSrvRequest = std::make_shared<ocs2_msgs::srv::Reset::Request>();
        resetSrvRequest->reset = true;
        resetSrvRequest->target_trajectories =
            ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(initTargetTrajectories);

        while (!mpcResetServiceClient_->wait_for_service(std::chrono::seconds(5)) && rclcpp::ok())
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(),"Failed to call service to reset MPC, retrying...");
        }

        mpcResetServiceClient_->async_send_request(resetSrvRequest);
        RCLCPP_INFO_STREAM(node_->get_logger(), "MPC node has been reset.");
    }

    void MRT_ROS_Interface::setCurrentObservation(const ocs2::SystemObservation &currentObservation)
    {
        mpcObservationMsg_ = ocs2::ros_msg_conversions::createObservationMsg(currentObservation);
        if (mpcObservationPublisher_->trylock())
        {
            mpcObservationPublisher_->msg_ = mpcObservationMsg_;
            mpcObservationPublisher_->unlockAndPublish();
        }
    }

    void MRT_ROS_Interface::readPolicyMsg(const ocs2_msgs::msg::MpcFlattenedController &msg, 
                                          ocs2::CommandData &commandData,
                                          ocs2::PrimalSolution &primalSolution, 
                                          ocs2::PerformanceIndex &performanceIndices)
    {
        commandData.mpcInitObservation_ =
            ocs2::ros_msg_conversions::readObservationMsg(msg.init_observation);
        commandData.mpcTargetTrajectories_ =
            ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(
                msg.plan_target_trajectories);
        performanceIndices =
            ocs2::ros_msg_conversions::readPerformanceIndicesMsg(msg.performance_indices);

        const size_t N = msg.time_trajectory.size();
        if (N == 0)
        {
            throw std::runtime_error(
                "[MRT_ROS_Interface::readPolicyMsg] controller message is empty!");
        }
        if (msg.state_trajectory.size() != N && msg.input_trajectory.size() != N)
        {
            throw std::runtime_error(
                "[MRT_ROS_Interface::readPolicyMsg] state and input trajectories must "
                "have same length!");
        }
        if (msg.data.size() != N)
        {
            throw std::runtime_error(
                "[MRT_ROS_Interface::readPolicyMsg] Data has the wrong length!");
        }

        primalSolution.clear();

        primalSolution.modeSchedule_ = ocs2::ros_msg_conversions::readModeScheduleMsg(msg.mode_schedule);

        ocs2::size_array_t stateDim(N);
        ocs2::size_array_t inputDim(N);
        primalSolution.timeTrajectory_.reserve(N);
        primalSolution.stateTrajectory_.reserve(N);
        primalSolution.inputTrajectory_.reserve(N);
        for (size_t i = 0; i < N; i++)
        {
            stateDim[i] = msg.state_trajectory[i].value.size();
            inputDim[i] = msg.input_trajectory[i].value.size();
            primalSolution.timeTrajectory_.emplace_back(msg.time_trajectory[i]);
            primalSolution.stateTrajectory_.emplace_back(
                Eigen::Map<const Eigen::VectorXf>(msg.state_trajectory[i].value.data(),
                                                  stateDim[i])
                    .cast<ocs2::scalar_t>());
            primalSolution.inputTrajectory_.emplace_back(
                Eigen::Map<const Eigen::VectorXf>(msg.input_trajectory[i].value.data(),
                                                  inputDim[i])
                    .cast<ocs2::scalar_t>());
        }

        primalSolution.postEventIndices_.reserve(msg.post_event_indices.size());
        for (auto ind : msg.post_event_indices)
        {
            primalSolution.postEventIndices_.emplace_back(static_cast<size_t>(ind));
        }

        std::vector<std::vector<float> const *> controllerDataPtrArray(N, nullptr);
        for (size_t i = 0; i < N; i++)
        {
            controllerDataPtrArray[i] = &(msg.data[i].data);
        }

        // instantiate the correct controller
        switch (msg.controller_type)
        {
        case ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_FEEDFORWARD:
        {
            auto controller = ocs2::FeedforwardController::unFlatten(
                primalSolution.timeTrajectory_, controllerDataPtrArray);
            primalSolution.controllerPtr_.reset(
                new ocs2::FeedforwardController(std::move(controller)));
            break;
        }
        case ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_LINEAR:
        {
            auto controller = ocs2::LinearController::unFlatten(
                stateDim, inputDim, primalSolution.timeTrajectory_,
                controllerDataPtrArray);
            primalSolution.controllerPtr_.reset(
                new ocs2::LinearController(std::move(controller)));
            break;
        }
        default:
            throw std::runtime_error(
                "[MRT_ROS_Interface::readPolicyMsg] Unknown controllerType!");
        }
    }

    void MRT_ROS_Interface::mpcPolicyCallback(const ocs2_msgs::msg::MpcFlattenedController::ConstSharedPtr &msg)
    {
        // read new policy and command from msg
        auto commandPtr = std::make_unique<ocs2::CommandData>();
        auto primalSolutionPtr = std::make_unique<ocs2::PrimalSolution>();
        auto performanceIndicesPtr = std::make_unique<ocs2::PerformanceIndex>();
        readPolicyMsg(*msg, *commandPtr, *primalSolutionPtr, *performanceIndicesPtr);

        this->moveToBuffer(std::move(commandPtr), std::move(primalSolutionPtr),
                           std::move(performanceIndicesPtr));
    }

    void MRT_ROS_Interface::launchNodes()
    {
        this->reset();
        
        // display
        RCLCPP_INFO_STREAM(node_->get_logger(), "MRT node is setting up ...");

        // observation publisher
        auto obs_pub = node_->create_publisher<ocs2_msgs::msg::MpcObservation>(
            topicPrefix_ + "_mpc_observation", 1);
        mpcObservationPublisher_ = std::make_shared<realtime_tools::RealtimePublisher<ocs2_msgs::msg::MpcObservation>>(obs_pub);

        // policy subscriber
        mpcPolicySubscriber_ = node_->create_subscription<ocs2_msgs::msg::MpcFlattenedController>(
            topicPrefix_ + "_mpc_policy", // topic name
            1,                            // queue length
            std::bind(&MRT_ROS_Interface::mpcPolicyCallback, this, std::placeholders::_1));

        // MPC reset service client
        mpcResetServiceClient_ = node_->create_client<ocs2_msgs::srv::Reset>(topicPrefix_ + "_mpc_reset");

        // display
        RCLCPP_INFO_STREAM(node_->get_logger(), "MRT node is ready.");
    }
} // namespace legged

namespace legged_robot
{
    controller_interface::CallbackReturn legged_mrt::on_init()
    {
        // Initialize parameters
        joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
        if (joint_names_.size() != 12u)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(), "Expected 12 joint names, got %zu", joint_names_.size());
            return CallbackReturn::ERROR;
        }

        robotName = auto_declare<std::string>("robotName", "legged_robot");
        taskFile = auto_declare<std::string>("taskFile", "");
        urdfFile = auto_declare<std::string>("urdfFile", "");
        referenceFile = auto_declare<std::string>("referenceFile", "");

        visualization_enable_ = auto_declare<bool>("visualization_enable", true);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration legged_mrt::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(12 + 12 + 12);

        config.names.push_back("estimation/orientation_roll");
        config.names.push_back("estimation/orientation_pitch");
        config.names.push_back("estimation/orientation_yaw");

        config.names.push_back("estimation/position_x");
        config.names.push_back("estimation/position_y");
        config.names.push_back("estimation/position_z");

        config.names.push_back("estimation/linear_x");
        config.names.push_back("estimation/linear_y");
        config.names.push_back("estimation/linear_z");

        config.names.push_back("estimation/angular_x");
        config.names.push_back("estimation/angular_y");
        config.names.push_back("estimation/angular_z");

        for (const auto &joint_name : joint_names_)
        {
            config.names.push_back(joint_name + "/position");
        }
        // 申请关节位置接口
        for (const auto &joint_name : joint_names_)
        {
            config.names.push_back(joint_name + "/velocity");
        }
        // 申请关节速度接口

        return config;
    }

    controller_interface::InterfaceConfiguration legged_mrt::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::NONE;
        return config;
    }

    std::vector<hardware_interface::StateInterface> legged_mrt::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        ExportoptimizedState(&state_interfaces);
        ExportoptimizedInput(&state_interfaces);
        ExportRbdState(&state_interfaces);
        state_interfaces.push_back(
            hardware_interface::StateInterface("desiredMode", "contact_state", &plannedMode_));
        return state_interfaces;
    }

    controller_interface::CallbackReturn legged_mrt::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;

        leggedInterface_ = legged_robot::LeggedInterfaceProvider::get(taskFile, urdfFile, referenceFile);
        ocs2::CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());

        // Visualization
        if (visualization_enable_)
        {
            ocs2::CentroidalModelPinocchioMapping pinocchioMapping(
                leggedInterface_->getCentroidalModelInfo());
            ocs2::PinocchioEndEffectorKinematics endEffectorKinematics(
                leggedInterface_->getPinocchioInterface(),
                pinocchioMapping,
                leggedInterface_->modelSettings().contactNames3DoF);
            leggedRobotVisualizer_ = std::make_shared<ocs2::legged_robot::LeggedRobotVisualizer>(
                leggedInterface_->getPinocchioInterface(),
                leggedInterface_->getCentroidalModelInfo(),
                endEffectorKinematics, get_node());
        }

        // MRT
        mrt_ = std::make_shared<legged::MRT_ROS_Interface>(robotName, get_node());
        mrt_->initRollout(&leggedInterface_->getRollout());
        mrt_->launchNodes();

        // Resize state and input
        info_ = leggedInterface_->getCentroidalModelInfo();
        rbdState_.resize(info_.generalizedCoordinatesNum * 2);
        rbdState_.setZero();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn legged_mrt::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        // Initial state
        ocs2::SystemObservation initObservation;
        initObservation.state = leggedInterface_->getInitialState();
        initObservation.input =
            ocs2::vector_t::Zero(leggedInterface_->getCentroidalModelInfo().inputDim);
        initObservation.mode = ocs2::legged_robot::ModeNumber::STANCE;

        // Initial command
        ocs2::TargetTrajectories initTargetTrajectories({0.0}, {initObservation.state},
                                                        {initObservation.input});

        RCLCPP_INFO_STREAM(get_node()->get_logger(), "Waiting for the initial policy ...");

        // Reset MPC node
        mrt_->resetMpcNode(initTargetTrajectories);

        // Wait for the initial policy
        while (!mrt_->initialPolicyReceived() && rclcpp::ok())
        {
            rclcpp::sleep_for(std::chrono::milliseconds(10));
            mrt_->setCurrentObservation(initObservation);
        }
        RCLCPP_INFO_STREAM(get_node()->get_logger(), "Initial policy has been received.");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type legged_mrt::update_reference_from_subscribers(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type legged_mrt::update_and_write_commands(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)period;

        currentObservation_.time = time.seconds();
        ocs2::legged_robot::vector3_t zyx, angularVel, pos, linearVel;
        ocs2::vector_t jointPos, jointVel;
        jointPos.resize(12);
        jointVel.resize(12);

        for (uint8_t i = 0; i < 3; i++)
        {
            zyx(i) = state_interfaces_[i].get_optional().value();
            pos(i) = state_interfaces_[i + 3].get_optional().value();
            linearVel(i) = state_interfaces_[i + 6].get_optional().value();
            angularVel(i) = state_interfaces_[i + 9].get_optional().value();
        }

        for (uint8_t i = 0; i < 12; i++)
        {
            jointPos(i) = state_interfaces_[12 + i].get_optional().value();
            jointVel(i) = state_interfaces_[24 + i].get_optional().value();
        }

        rbdState_.segment<3>(0) = zyx;
        rbdState_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;

        rbdState_.segment<3>(3) = pos;
        rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;

        rbdState_.segment(6, info_.actuatedDofNum) = jointPos;
        rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = jointVel;
        // 从观测值更新刚体动力学模型状态

        ocs2::scalar_t yawLast = currentObservation_.state(9);
        currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(rbdState_);
        currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
        currentObservation_.mode = static_cast<size_t>(state_interfaces_.back().get_optional().value());
        // 从刚体动力学模型更新MPC观测值

        mrt_->setCurrentObservation(currentObservation_);
        mrt_->updatePolicy();

        size_t plannedMode = 0; // The mode that is active at the time the policy is evaluated at.
        mrt_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState_, optimizedInput_, plannedMode);
        plannedMode_ = static_cast<double>(plannedMode);
        // 从MPC获取优化结果

        if (visualization_enable_)
        {
            leggedRobotVisualizer_->update(currentObservation_, mrt_->getPolicy(), mrt_->getCommand());
        }
        // Visualization

        return controller_interface::return_type::OK;
    }

    void legged_mrt::ExportoptimizedState(std::vector<hardware_interface::StateInterface> *state_interfaces)
    {
        const std::vector<std::string> names = {
            "vcom_x", "vcom_y", "vcom_z",
            // linear momentum
            "L_x", "L_y", "L_z",
            // angular momentum
            "p_base_x", "p_base_y", "p_base_z",
            // base postion
            "theta_base_z", "theta_base_y", "theta_base_x",
            // base orientation
            "LF_HAA", "LF_HFE", "LF_KFE",
            // left front leg positions
            "LH_HAA", "LH_HFE", "LH_KFE",
            // left hind leg positions
            "RF_HAA", "RF_HFE", "RF_KFE",
            // right front leg positions
            "RH_HAA", "RH_HFE", "RH_KFE"
            // right hind leg positions
        };

        state_interfaces->reserve(names.size());
        for (size_t i = 0; i < names.size(); i++)
        {
            state_interfaces->emplace_back(
                hardware_interface::StateInterface("optimizedState", names[i], &optimizedState_(i)));
        }
    }

    void legged_mrt::ExportoptimizedInput(std::vector<hardware_interface::StateInterface> *state_interfaces)
    {
        const std::vector<std::string> names = {
            "left_front_force_x", "left_front_force_y", "left_front_force_z",
            "right_front_force_x", "right_front_force_y", "right_front_force_z",
            "left_hind_force_x", "left_hind_force_y", "left_hind_force_z",
            "right_hind_force_x", "right_hind_force_y", "right_hind_force_z",
            // contact forces
            "LF_x", "LF_y", "LF_z",
            "LH_x", "LH_y", "LH_z",
            "RF_x", "RF_y", "RF_z",
            "RH_x", "RH_y", "RH_z"
            // feet contact forces
        };

        state_interfaces->reserve(names.size());
        for (size_t i = 0; i < names.size(); i++)
        {
            state_interfaces->emplace_back(
                hardware_interface::StateInterface("optimizedInput", names[i], &optimizedInput_(i)));
        }
    }

    void legged_mrt::ExportRbdState(std::vector<hardware_interface::StateInterface> *state_interfaces)
    {
        const std::vector<std::string> names = {
            "base_roll", "base_pitch", "base_yaw",
            // base orientation
            "base_pos_x", "base_pos_y", "base_pos_z",
            // base position
            "LF_HAA_pos", "LF_HFE_pos", "LF_KFE_pos",
            // left front leg joint positions
            "LH_HAA_pos", "LH_HFE_pos", "LH_KFE_pos",
            // left hind leg joint positions
            "RF_HAA_pos", "RF_HFE_pos", "RF_KFE_pos",
            // right front leg joint positions
            "RH_HAA_pos", "RH_HFE_pos", "RH_KFE_pos",
            // right hind leg joint positions

            "base_angular_vel_x", "base_angular_vel_y", "base_angular_vel_z",
            // base angular velocity
            "base_linear_vel_x", "base_linear_vel_y", "base_linear_vel_z",
            // base linear velocity
            "LF_HAA_vel", "LF_HFE_vel", "LF_KFE_vel",
            // left front leg joint velocities
            "LH_HAA_vel", "LH_HFE_vel", "LH_KFE_vel",
            // left hind leg joint velocities
            "RF_HAA_vel", "RF_HFE_vel", "RF_KFE_vel",
            // right front leg joint velocities
            "RH_HAA_vel", "RH_HFE_vel", "RH_KFE_vel"
            // right hind leg joint velocities
        };

        state_interfaces->reserve(names.size());
        for (size_t i = 0; i < names.size(); i++)
        {
            state_interfaces->emplace_back(
                hardware_interface::StateInterface("rbdState", names[i], &rbdState_(i)));
        }
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(legged_robot::legged_mrt, controller_interface::ChainableControllerInterface)
