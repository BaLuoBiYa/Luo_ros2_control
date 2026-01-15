#pragma once
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/sensor_interface.hpp>

#include <std_msgs/msg/bool.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

namespace legged
{
    class contactSim : public hardware_interface::SensorInterface
    {
    public:
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &pre) override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    private:
        std::string contacts_topic_;
        std::string tip_names_[4] = {"LF", "LH", "RF", "RH"};

        realtime_tools::RealtimeThreadSafeBox<std_msgs::msg::Bool> received_contacts_msg_[4];
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_subscriber_[4];

        bool contact_flag_[4];
    };
} // namespace legged