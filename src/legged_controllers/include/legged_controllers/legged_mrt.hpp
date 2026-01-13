/******************************************************************************
A rewrit version of ocs2::MRT_ROS_Interface to fit legged robot controller
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
#include "ocs2_ros_interfaces/common/RosMsgConversions.h"

#include <realtime_tools/realtime_publisher.hpp>

namespace legged {
    /**
     * This class implements MRT (Model Reference Tracking) communication interface
     * using ROS.
     */
    class MRT_ROS_Interface : public ocs2::MRT_BASE {
    public:
        /**
         * Constructor
         *
         * @param [in] topicPrefix: The prefix defines the names for: observation's
         * publishing topic "topicPrefix_mpc_observation", policy's receiving topic
         * "topicPrefix_mpc_policy", and MPC reset service "topicPrefix_mpc_reset".
         * @param [in] mrtTransportHints: ROS transmission protocol.
         */
        explicit MRT_ROS_Interface(std::string topicPrefix ,const rclcpp_lifecycle::LifecycleNode::SharedPtr node);

        /**
         * Destructor
         */
        ~MRT_ROS_Interface() override;

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


#include <controller_interface/chainable_controller_interface.hpp>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include "ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h"
#include "legged_controllers/legged_InterfaceProvider.hpp"

#include <angles/angles.h>

namespace legged_robot{
    class legged_mrt:public controller_interface::ChainableControllerInterface{
        protected:
            // Parameters
            std::string robotName;
            std::string taskFile;
            std::string urdfFile;
            std::string referenceFile;
            std::vector<std::string> joint_names_;
            bool visualization_enable_;

            // Interface
            std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface> leggedInterface_;
            std::shared_ptr<ocs2::legged_robot::LeggedRobotVisualizer> leggedRobotVisualizer_;

            // MRT
            std::shared_ptr<legged::MRT_ROS_Interface> mrt_;

            // State Estimation
            ocs2::SystemObservation currentObservation_;
            ocs2::CentroidalModelInfo info_;
            ocs2::vector_t optimizedState_, optimizedInput_;
            double plannedMode_;
            ocs2::vector_t rbdState_;
            std::shared_ptr<ocs2::CentroidalModelRbdConversions> rbdConversions_;

            void ExportoptimizedState(std::vector<hardware_interface::StateInterface> * state_interfaces);
            void ExportoptimizedInput(std::vector<hardware_interface::StateInterface> * state_interfaces);
            void ExportRbdState(std::vector<hardware_interface::StateInterface> * state_interfaces);

        public:
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;
            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

            // controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

            std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;
            // std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
            // bool on_set_chained_mode(bool chained_mode) override;

            controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            controller_interface::return_type update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    }; // class legged_mrt
}// namespace legged_robot
