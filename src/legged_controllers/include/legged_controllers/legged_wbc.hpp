#pragma once

#include <controller_interface/chainable_controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
// #include <ocs2_core/misc/Benchmark.h>
// #include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
// #include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
// #include <ocs2_mpc/MPC_MRT_Interface.h>

#include "ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h"
#include "ocs2_legged_robot/wbc/HierarchicalWbc.h"
#include "ocs2_legged_robot/wbc/WbcBase.h"
#include "ocs2_legged_robot/wbc/WeightedWbc.h"

#include "legged_controllers/legged_mrt.hpp"
#include "legged_controllers/legged_safty.hpp"

namespace legged_robot {
    class legged_wbc:public controller_interface::ChainableControllerInterface
    {
    protected:
        union StateEstimate_t{
            double data[12];
            struct {
                struct position{
                    double x;
                    double y;
                    double z;
                }position;
                struct orientation{
                    double roll;
                    double pitch;
                    double yaw;
                }orientation;
                struct linear_velocity{
                    double x;
                    double y;
                    double z;
                }linear_velocity;
                struct angular_velocity{
                    double x;
                    double y;
                    double z;
                }angular_velocity;
            };
        };

        // Parameters
        std::string robotName;
        std::string taskFile;
        std::string urdfFile;
        std::string referenceFile;
        std::vector<std::string> joint_names_;
        bool visualization_enable_;

        // Interface
        std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface> leggedInterface_;
        std::shared_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematicsPtr_;
        std::shared_ptr<ocs2::legged_robot::LeggedRobotVisualizer> leggedRobotVisualizer_;

        // WBC
        std::shared_ptr<legged::WbcBase> wbc_;
        std::shared_ptr<legged::SafetyChecker> safetyChecker_;

        // MRT
        std::shared_ptr<legged::MRT_ROS_Interface> mrt_;

        // State Estimation
        ocs2::SystemObservation currentObservation_;
        ocs2::vector_t measuredRbdState_;
        // std::shared_ptr<StateEstimateBase> stateEstimate_;
        StateEstimate_t stateEstimation_;
        std::shared_ptr<ocs2::CentroidalModelRbdConversions> rbdConversions_;

    public:
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        // controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        // std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;
        // std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
        // bool on_set_chained_mode(bool chained_mode) override;

        controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        controller_interface::return_type update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    };// class legged_wbc
    
} // namespace legged_controllers