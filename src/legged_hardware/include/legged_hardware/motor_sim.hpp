# pragma once
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/system_interface.hpp"

namespace legged
{
    class MotorSim : public hardware_interface::SystemInterface
    {
    public:
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &pre) override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    private:

    };
} // namespace legged