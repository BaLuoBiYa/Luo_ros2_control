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

        resetTopics_[0] = info_.hardware_parameters["reset_topic_LF"];
        resetTopics_[1] = info_.hardware_parameters["reset_topic_LH"];
        resetTopics_[2] = info_.hardware_parameters["reset_topic_RF"];
        resetTopics_[3] = info_.hardware_parameters["reset_topic_RH"];

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
        for (size_t i =0;i<4;i++){
            receivedContactsMsg_[i].try_set(false);
        } 
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn contactSim::on_deactivate(const rclcpp_lifecycle::State &pre) {
        (void)pre;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type contactSim::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)period;
        (void)time;
        for (size_t i = 0; i < 4; i++) {
            set_state<bool>("contact/" + tipNames_[i], receivedContactsMsg_[i].try_get().value_or(false));
        }
        resetSensor();

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type contactSim::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)time;
        (void)period;
        return hardware_interface::return_type::OK;
    }

    void contactSim::LFcallBack(const gz::msgs::Boolean &msg) {
        receivedContactsMsg_[0].try_set(msg.data());
    }
    void contactSim::LHcallBack(const gz::msgs::Boolean &msg) {
        receivedContactsMsg_[1].try_set(msg.data());
    }
    void contactSim::RFcallBack(const gz::msgs::Boolean &msg) {
        receivedContactsMsg_[2].try_set(msg.data());
    }
    void contactSim::RHcallBack(const gz::msgs::Boolean &msg) {
        receivedContactsMsg_[3].try_set(msg.data());
    }

    void contactSim::resetSensor()
    {
        gz::msgs::Boolean req;
        req.set_data(true);
        for (size_t i=0;i<4;i++){
            receivedContactsMsg_[i].try_set(false);
            node_.Request(resetTopics_[i],req);
        }
    }
} // namespace legged

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(legged::contactSim, gz_ros2_control::GazeboSimSystemInterface)