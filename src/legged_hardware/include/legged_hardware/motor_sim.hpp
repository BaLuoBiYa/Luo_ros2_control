# pragma once
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

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

        // std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        // std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    private:
        
        realtime_tools::RealtimePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_publisher_;
        realtime_tools::RealtimeThreadSafeBox<sensor_msgs::msg::JointState> received_msg_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
        trajectory_msgs::msg::JointTrajectory joint_command_;

        std::string joint_command_topic_, joint_state_topic_;
        std::string joint_names_[12] = {"LF_HAA", "LF_HFE", "LF_KFE",
                                       "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE",
                                       "RH_HAA", "RH_HFE", "RH_KFE"};

        double current_position_[12];
        double current_velocity_[12];
        double current_effort_[12];

        double command_position_[12];
        double command_velocity_[12];
        double command_effort_[12];

        int32_t reach_time_sec_;
        uint32_t reach_time_nanosec_;
    };
} // namespace legged