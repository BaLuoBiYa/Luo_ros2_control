#pragma once
#include <hardware_interface/sensor_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace legged {
    class ImuSim : public hardware_interface::SensorInterface {
      public:
        hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &pre) override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      private:
        std::string imuTopic_;
        std::string imuName_;

        realtime_tools::RealtimeThreadSafeBox<sensor_msgs::msg::Imu> receivedImuMsg_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscriber_;
        // double orientation_[4];
        // double angular_velocity_[3];
        // double linear_acceleration_[3];
    };
} // namespace legged