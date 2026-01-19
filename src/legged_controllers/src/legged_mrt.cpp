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

namespace legged {
    mrtInterface::mrtInterface(std::string topicPrefix, const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : topicPrefix_(topicPrefix), node_(node) {}

    mrtInterface::~mrtInterface() {}

    void mrtInterface::resetMpcNode(const ocs2::TargetTrajectories &initTargetTrajectories) {
        this->reset();

        const auto resetSrvRequest = std::make_shared<ocs2_msgs::srv::Reset::Request>();
        resetSrvRequest->reset = true;
        resetSrvRequest->target_trajectories =
            ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(initTargetTrajectories);

        while (!mpcResetServiceClient_->wait_for_service(std::chrono::seconds(5)) && rclcpp::ok()) {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to call service to reset MPC, retrying...");
        }

        mpcResetServiceClient_->async_send_request(resetSrvRequest);
        RCLCPP_INFO_STREAM(node_->get_logger(), "MPC node has been reset.");
    }

    void mrtInterface::setCurrentObservation(const ocs2::SystemObservation &currentObservation) {
        mpcObservationMsg_ = ocs2::ros_msg_conversions::createObservationMsg(currentObservation);
        if (mpcObservationPublisher_->trylock()) {
            mpcObservationPublisher_->msg_ = mpcObservationMsg_;
            mpcObservationPublisher_->unlockAndPublish();
        }
    }

    void mrtInterface::readPolicyMsg(const ocs2_msgs::msg::MpcFlattenedController &msg, ocs2::CommandData &commandData,
                                     ocs2::PrimalSolution &primalSolution, ocs2::PerformanceIndex &performanceIndices) {
        commandData.mpcInitObservation_ = ocs2::ros_msg_conversions::readObservationMsg(msg.init_observation);
        commandData.mpcTargetTrajectories_ =
            ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(msg.plan_target_trajectories);
        performanceIndices = ocs2::ros_msg_conversions::readPerformanceIndicesMsg(msg.performance_indices);

        const size_t N = msg.time_trajectory.size();
        if (N == 0) {
            throw std::runtime_error("[mrtInterface::readPolicyMsg] controller message is empty!");
        }
        if (msg.state_trajectory.size() != N && msg.input_trajectory.size() != N) {
            throw std::runtime_error("[mrtInterface::readPolicyMsg] state and input trajectories must "
                                     "have same length!");
        }
        if (msg.data.size() != N) {
            throw std::runtime_error("[mrtInterface::readPolicyMsg] Data has the wrong length!");
        }

        primalSolution.clear();

        primalSolution.modeSchedule_ = ocs2::ros_msg_conversions::readModeScheduleMsg(msg.mode_schedule);

        ocs2::size_array_t stateDim(N);
        ocs2::size_array_t inputDim(N);
        primalSolution.timeTrajectory_.reserve(N);
        primalSolution.stateTrajectory_.reserve(N);
        primalSolution.inputTrajectory_.reserve(N);
        for (size_t i = 0; i < N; i++) {
            stateDim[i] = msg.state_trajectory[i].value.size();
            inputDim[i] = msg.input_trajectory[i].value.size();
            primalSolution.timeTrajectory_.emplace_back(msg.time_trajectory[i]);
            primalSolution.stateTrajectory_.emplace_back(
                Eigen::Map<const Eigen::VectorXf>(msg.state_trajectory[i].value.data(), stateDim[i])
                    .cast<ocs2::scalar_t>());
            primalSolution.inputTrajectory_.emplace_back(
                Eigen::Map<const Eigen::VectorXf>(msg.input_trajectory[i].value.data(), inputDim[i])
                    .cast<ocs2::scalar_t>());
        }

        primalSolution.postEventIndices_.reserve(msg.post_event_indices.size());
        for (auto ind : msg.post_event_indices) {
            primalSolution.postEventIndices_.emplace_back(static_cast<size_t>(ind));
        }

        std::vector<std::vector<float> const *> controllerDataPtrArray(N, nullptr);
        for (size_t i = 0; i < N; i++) {
            controllerDataPtrArray[i] = &(msg.data[i].data);
        }

        // instantiate the correct controller
        switch (msg.controller_type) {
        case ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_FEEDFORWARD: {
            auto controller =
                ocs2::FeedforwardController::unFlatten(primalSolution.timeTrajectory_, controllerDataPtrArray);
            primalSolution.controllerPtr_.reset(new ocs2::FeedforwardController(std::move(controller)));
            break;
        }
        case ocs2_msgs::msg::MpcFlattenedController::CONTROLLER_LINEAR: {
            auto controller = ocs2::LinearController::unFlatten(stateDim, inputDim, primalSolution.timeTrajectory_,
                                                                controllerDataPtrArray);
            primalSolution.controllerPtr_.reset(new ocs2::LinearController(std::move(controller)));
            break;
        }
        default:
            throw std::runtime_error("[mrtInterface::readPolicyMsg] Unknown controllerType!");
        }
    }

    void mrtInterface::mpcPolicyCallback(const ocs2_msgs::msg::MpcFlattenedController::ConstSharedPtr &msg) {
        // read new policy and command from msg
        auto commandPtr = std::make_unique<ocs2::CommandData>();
        auto primalSolutionPtr = std::make_unique<ocs2::PrimalSolution>();
        auto performanceIndicesPtr = std::make_unique<ocs2::PerformanceIndex>();
        readPolicyMsg(*msg, *commandPtr, *primalSolutionPtr, *performanceIndicesPtr);

        this->moveToBuffer(std::move(commandPtr), std::move(primalSolutionPtr), std::move(performanceIndicesPtr));
    }

    void mrtInterface::launchNodes() {
        this->reset();

        // display
        RCLCPP_INFO_STREAM(node_->get_logger(), "MRT node is setting up ...");

        // observation publisher
        auto obs_pub = node_->create_publisher<ocs2_msgs::msg::MpcObservation>(topicPrefix_ + "_mpc_observation", 1);
        mpcObservationPublisher_ =
            std::make_shared<realtime_tools::RealtimePublisher<ocs2_msgs::msg::MpcObservation>>(obs_pub);

        // policy subscriber
        mpcPolicySubscriber_ = node_->create_subscription<ocs2_msgs::msg::MpcFlattenedController>(
            topicPrefix_ + "_mpc_policy", // topic name
            1,                            // queue length
            std::bind(&mrtInterface::mpcPolicyCallback, this, std::placeholders::_1));

        // MPC reset service client
        mpcResetServiceClient_ = node_->create_client<ocs2_msgs::srv::Reset>(topicPrefix_ + "_mpc_reset");

        // display
        RCLCPP_INFO_STREAM(node_->get_logger(), "MRT node is ready.");
    }
} // namespace legged
