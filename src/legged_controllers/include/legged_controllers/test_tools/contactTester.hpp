#pragma once

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/string.hpp>

namespace test_tools{
    class contact_tester:public controller_interface::ControllerInterface{
        protected:
            // Parameters
            std::vector<std::string> contactNames_;

            // Publisher
            realtime_tools::RealtimePublisher<std_msgs::msg::String>::UniquePtr statePub_;

        public:
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;
            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

            // controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

            controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    }; // class legged_mrt
}// namespace legged