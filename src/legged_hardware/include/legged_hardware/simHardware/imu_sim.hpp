#pragma once
#include <gz/msgs/imu.pb.h>
#include <gz/transport/Node.hh>
#include <gz_ros2_control/gz_system_interface.hpp>

#include <realtime_tools/realtime_thread_safe_box.hpp>

namespace legged {
    class ImuSim : public gz_ros2_control::GazeboSimSystemInterface {
      public:
        hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &pre) override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        bool initSim(rclcpp::Node::SharedPtr &model_nh, std::map<std::string, sim::Entity> &joints,
                             const hardware_interface::HardwareInfo &hardware_info, sim::EntityComponentManager &_ecm,
                             unsigned int update_rate) override;

      private:
        std::string imuTopic_;
        std::string imuName_;
        gz::transport::Node node_;
        realtime_tools::RealtimeThreadSafeBox<gz::msgs::IMU> receivedImuMsg_;
        void imuCallback(const gz::msgs::IMU &msg);
        // double angular_velocity_[3];
        // double linear_acceleration_[3];
    };
} // namespace legged