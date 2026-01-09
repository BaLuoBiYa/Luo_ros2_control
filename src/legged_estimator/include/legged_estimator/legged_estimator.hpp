#pragma once

#include <rclcpp/rclcpp.hpp>

#include <controller_interface/chainable_controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_mpc/SystemObservation.h>
#include "legged_estimator/StateEstimateBase.hpp"

namespace legged_robot
{
    class legged_estimator : public controller_interface::ChainableControllerInterface
    {
    protected:
        ocs2::SystemObservation currentObservation_;
        ocs2::vector_t measuredRbdState_;
        std::shared_ptr<legged_robot::StateEstimateBase> stateEstimate_;
        std::shared_ptr<ocs2::CentroidalModelRbdConversions> rbdConversions_;
        ocs2::vector_t rbdState_;
        
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

        controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        controller_interface::return_type update_and_write_commands(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    }; // class legged_estimator
} // namespace legged_controllers