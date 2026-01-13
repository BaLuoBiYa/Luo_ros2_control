/******************************************************************************
A rewrit version of ocs2::mrtInterface to fit legged robot controller
******************************************************************************/

#pragma once

#include <condition_variable>
#include <string>
// #include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/logger.hpp>

// MPC messages
#include <ocs2_mpc/MRT_BASE.h>
#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
#include <ocs2_msgs/srv/reset.hpp>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <realtime_tools/realtime_publisher.hpp>

namespace legged {
    /**
     * This class implements MRT (Model Reference Tracking) communication interface
     * using ROS.
     */
    class mrtInterface : public ocs2::MRT_BASE {
    public:
        /**
         * Constructor
         *
         * @param [in] topicPrefix: The prefix defines the names for: observation's
         * publishing topic "topicPrefix_mpc_observation", policy's receiving topic
         * "topicPrefix_mpc_policy", and MPC reset service "topicPrefix_mpc_reset".
         * @param [in] mrtTransportHints: ROS transmission protocol.
         */
        explicit mrtInterface(std::string topicPrefix ,const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

        /**
         * Destructor
         */
        ~mrtInterface() override;

        void resetMpcNode(const ocs2::TargetTrajectories &initTargetTrajectories) override;

        /**
         * Launches the ROS publishers and subscribers to communicate with the MPC
         * node.
         * @param node
         */
        void launchNodes();

        void setCurrentObservation(
            const ocs2::SystemObservation &currentObservation) override;

    private:
        /**
         * Callback method to receive the MPC policy as well as the mode sequence.
         * It only updates the policy variables with suffix (*Buffer_) variables.
         *
         * @param [in] msg: A constant pointer to the message
         */
        void mpcPolicyCallback(
            const ocs2_msgs::msg::MpcFlattenedController::ConstSharedPtr &msg);

        /**
         * Helper function to read a MPC policy message.
         *
         * @param [in] msg: A constant pointer to the message
         * @param [out] commandData: The MPC command data
         * @param [out] primalSolution: The MPC policy data
         * @param [out] performanceIndices: The MPC performance indices data
         */
        static void readPolicyMsg(const ocs2_msgs::msg::MpcFlattenedController &msg,
                                  ocs2::CommandData &commandData,
                                  ocs2::PrimalSolution &primalSolution,
                                  ocs2::PerformanceIndex &performanceIndices);

    private:
        std::string topicPrefix_;
        const rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

        // Publishers and subscribers
        std::shared_ptr<realtime_tools::RealtimePublisher<ocs2_msgs::msg::MpcObservation>> mpcObservationPublisher_;
        // rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr
        // mpcObservationPublisher_;
        rclcpp::Subscription<ocs2_msgs::msg::MpcFlattenedController>::SharedPtr
        mpcPolicySubscriber_;
        rclcpp::Client<ocs2_msgs::srv::Reset>::SharedPtr mpcResetServiceClient_;

        // ROS messages
        ocs2_msgs::msg::MpcObservation mpcObservationMsg_;
        ocs2_msgs::msg::MpcObservation mpcObservationMsgBuffer_;

        // rclcpp::executors::SingleThreadedExecutor callback_executor_;

        // Multi-threading for publishers
        // bool terminateThread_;
        // bool readyToPublish_;
        // std::thread publisherWorker_;
        // std::mutex publisherMutex_;
        // std::condition_variable msgReady_;
    };

} // namespace legged
