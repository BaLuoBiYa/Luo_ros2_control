#include "legged_hardware/contact_sim.hpp"

namespace legged {
    hardware_interface::CallbackReturn
    contactSim::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
        if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        contactsTopics_[0] = info_.hardware_parameters["contacts_topic_LF"];
        contactsTopics_[1] = info_.hardware_parameters["contacts_topic_LH"];
        contactsTopics_[2] = info_.hardware_parameters["contacts_topic_RF"];
        contactsTopics_[3] = info_.hardware_parameters["contacts_topic_RH"];

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    bool contactSim::initSim(rclcpp::Node::SharedPtr &model_nh, std::map<std::string, sim::Entity> &joints,
                             const hardware_interface::HardwareInfo &hardware_info, sim::EntityComponentManager &_ecm,
                             unsigned int update_rate) {
        (void)model_nh;
        (void)joints;
        (void)hardware_info;
        (void)_ecm;
        (void)update_rate;
        return true;
    }

    hardware_interface::CallbackReturn contactSim::on_configure(const rclcpp_lifecycle::State &pre) {
        (void)pre;
        node_.Subscribe(contactsTopics_[0], &contactSim::LFcallBack, this);
        node_.Subscribe(contactsTopics_[1], &contactSim::LHcallBack, this);
        node_.Subscribe(contactsTopics_[2], &contactSim::RFcallBack, this);
        node_.Subscribe(contactsTopics_[3], &contactSim::RHcallBack, this);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn contactSim::on_activate(const rclcpp_lifecycle::State &pre) {
        (void)pre;
        gz::msgs::Contacts emptyMsg;
        // 使用当前 ROS 时间填充 stamp
        auto now = rclcpp::Clock().now();
        emptyMsg.mutable_header()->mutable_stamp()->set_sec(static_cast<int32_t>(now.seconds()));
        emptyMsg.mutable_header()->mutable_stamp()->set_nsec(static_cast<int32_t>(now.nanoseconds() % 1000000000LL));
        for (size_t i =0;i<4;i++){
            receivedContactsMsg_[i].try_set(emptyMsg);
        } 
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn contactSim::on_deactivate(const rclcpp_lifecycle::State &pre) {
        (void)pre;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type contactSim::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)period;
        const std::string kObstacle = "Obstacle::link::collision";

        for (size_t i = 0; i < 4; i++) {
            auto msg = receivedContactsMsg_[i].try_get();
            if (msg == std::nullopt) {
                set_state<bool>("contact/" + tipNames_[i], false);
                continue;
            }

            double msg_time = msg.value().header().stamp().sec() + msg.value().header().stamp().nsec() * 1e-9;
            double age = time.seconds() - msg_time;

            if (age > 0.1) {
                set_state<bool>("contact/" + tipNames_[i], false);
            } else {
                set_state<bool>("contact/" + tipNames_[i], true);
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type contactSim::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)time;
        (void)period;
        return hardware_interface::return_type::OK;
    }

    void contactSim::LFcallBack(const gz::msgs::Contacts &msg) {
        receivedContactsMsg_[0].try_set(msg);
    }
    void contactSim::LHcallBack(const gz::msgs::Contacts &msg) {
        receivedContactsMsg_[1].try_set(msg);
    }
    void contactSim::RFcallBack(const gz::msgs::Contacts &msg) {
        receivedContactsMsg_[2].try_set(msg);
    }
    void contactSim::RHcallBack(const gz::msgs::Contacts &msg) {
        receivedContactsMsg_[3].try_set(msg);
    }
} // namespace legged

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(legged::contactSim, gz_ros2_control::GazeboSimSystemInterface)