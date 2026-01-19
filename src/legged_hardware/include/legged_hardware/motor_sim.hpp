#pragma once
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

namespace legged {
    class MotorSim : public hardware_interface::SystemInterface {
      public:
        hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &pre) override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        // std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      private:
        realtime_tools::RealtimePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr jointCommandPublisher_;
        realtime_tools::RealtimeThreadSafeBox<sensor_msgs::msg::JointState> receivedMsg_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber_;
        trajectory_msgs::msg::JointTrajectory jointCommand_;
        std::string jointCommandTopic_, jointStateTopic_;
        std::string jointNames_[12] = {"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};

        double currentPosition_[12];
        double currentVelocity_[12];
        double currentEffort_[12];

        int32_t reachTimeSec_;
        uint32_t reachTimeNanosec_;
    };
} // namespace legged