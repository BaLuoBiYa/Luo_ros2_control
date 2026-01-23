#pragma once
#include <gz/msgs/boolean.pb.h>
#include <gz/transport/Node.hh>
#include <gz_ros2_control/gz_system_interface.hpp>

#include <realtime_tools/realtime_thread_safe_box.hpp>

namespace legged {
    class contactSim : public gz_ros2_control::GazeboSimSystemInterface {
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

        // std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      private:
        std::string contactsTopics_[4];
        std::string resetTopics_[4];
        std::string tipNames_[4] = {"LF", "LH", "RF", "RH"};

        gz::transport::Node node_;
        realtime_tools::RealtimeThreadSafeBox<bool> receivedContactsMsg_[4];

        void LFcallBack(const gz::msgs::Boolean &msg);
        void LHcallBack(const gz::msgs::Boolean &msg);
        void RFcallBack(const gz::msgs::Boolean &msg);
        void RHcallBack(const gz::msgs::Boolean &msg);

        void resetSensor();

        // bool contact_flag_[4];
    };
} // namespace legged