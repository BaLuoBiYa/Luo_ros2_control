#pragma once

#include <angles/angles.h>
#include <controller_interface/controller_interface.hpp>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_core/misc/LoadData.h>

#include "ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h"

#include "legged_controllers/SafetyChecker.hpp"
#include "legged_estimator/LinearKalmanFilter.hpp"
#include "legged_estimator/StateEstimateBase.hpp"
#include "legged_wbc/WbcBase.h"
#include "legged_wbc/WeightedWbc.h"

#include <memory>

namespace legged {
    class legged_controller : public controller_interface::ControllerInterface {
      protected:
        // Interface
        std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface> leggedInterface_;
        rclcpp::Node::SharedPtr mrtNode_;
        std::shared_ptr<ocs2::MRT_ROS_Interface> mrtInterface_;
        std::shared_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematicsPtr_;

        // State Estimation
        ocs2::SystemObservation currentObservation_;
        std::shared_ptr<StateEstimateBase> stateEstimate_;
        ocs2::vector_t measuredRbdState_;
        std::shared_ptr<ocs2::CentroidalModelRbdConversions> rbdConversions_;

        ocs2::vector_t jointPos_{12}, jointVel_{12};
        Eigen::Quaternion<scalar_t> quat_;
        ocs2::legged_robot::contact_flag_t contactFlag_;
        ocs2::legged_robot::vector3_t angularVel_, linearAccel_;

        // Whole Body Control
        std::shared_ptr<WbcBase> wbc_;
        std::shared_ptr<SafetyChecker> safetyChecker_;

        // Visualization
        rclcpp::Node::SharedPtr visualizeNode_;
        std::shared_ptr<ocs2::legged_robot::LeggedRobotVisualizer> robotVisualizer_;

        // Parameters
        std::vector<std::string> jointNames_;
        std::string imuName_;

        std::string robotName_;
        std::string taskFile_;
        std::string urdfFile_;
        std::string referenceFile_;

        double kp_, kd_,kf_;

        bool visualize_;
        bool verbose_;

        void updateEstimation(const rclcpp::Time &time, const rclcpp::Duration &period);

      public:
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        // controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    }; // class legged_mrt
} // namespace legged