#pragma once

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h"
#include "ocs2_legged_robot/wbc/HierarchicalWbc.h"
#include "ocs2_legged_robot/wbc/WbcBase.h"
#include "ocs2_legged_robot/wbc/WeightedWbc.h"
// #include "rclcpp/rclcpp.hpp"

// #include <legged_estimation/StateEstimateBase.h>

namespace legged_controllers {
    class legged_wbc:public controller_interface::ControllerInterface
    {
    protected:
        // Parameters
        const std::string robotName;
        const std::string taskFil;
        const std::string urdfFile;
        const std::string referenceFile;
        // Interface
        std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface> leggedInterface_;
        std::shared_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematicsPtr_;
        std::shared_ptr<ocs2::legged_robot::LeggedRobotVisualizer> leggedRobotVisualizer_;

        // WBC
        std::shared_ptr<legged::WbcBase> wbc_;

        // MRT
        std::shared_ptr<ocs2::MRT_ROS_Interface> mrt_;

        // State Estimation
        ocs2::SystemObservation currentObservation_;
        ocs2::vector_t measuredRbdState_;
        // std::shared_ptr<StateEstimateBase> stateEstimate_;
        // std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;
    public:
        legged_wbc() = default;
        ~legged_wbc();

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    };// class legged_wbc
    
} // namespace legged_controllers