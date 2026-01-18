#include "legged_hardware/contact_sim.hpp"

namespace legged
{
    hardware_interface::CallbackReturn contactSim::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SensorInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        contactsTopics_[0] = info_.hardware_parameters["contacts_topic_LF"];
        contactsTopics_[1] = info_.hardware_parameters["contacts_topic_LH"];
        contactsTopics_[2] = info_.hardware_parameters["contacts_topic_RF"];
        contactsTopics_[3] = info_.hardware_parameters["contacts_topic_RH"];

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn contactSim::on_configure(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        // for (int i = 0; i < 4; i++)
        // {
        //     contact_flag_[i] = false;
        // }

        for (size_t i = 0; i < 4; i++)
        {
            contactSubscriber_[i] = get_node()->create_subscription<ros_gz_interfaces::msg::Contacts>(
                contactsTopics_[i],
                1,
                [this, i](const ros_gz_interfaces::msg::Contacts::ConstSharedPtr &msg)
                {
                    this->receivedContactsMsg_[i].try_set(*msg);
                });
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn contactSim::on_activate(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        ros_gz_interfaces::msg::Contacts empty_msg;
        empty_msg.header.stamp.sec = 0;
        empty_msg.header.stamp.nanosec = 0;
        for (size_t i = 0; i < 4; i++)
        {
            receivedContactsMsg_[i].try_set(empty_msg);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn contactSim::on_deactivate(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type contactSim::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)period;
        const std::string kObstacle = "Obstacle::link::collision";

        for (size_t i = 0; i < 4; i++)
        {
            auto msg = receivedContactsMsg_[i].try_get();
            if (msg == std::nullopt)
            {
                set_state<bool>("contact/" + tipNames_[i], false);
                continue;
            }

            double msg_time = msg.value().header.stamp.sec + msg.value().header.stamp.nanosec * 1e-9;
            double age = time.seconds() - msg_time;

            if (age > 0.1)
            {
                set_state<bool>("contact/" + tipNames_[i], false);
            }
            else
            {
                set_state<bool>("contact/" + tipNames_[i], true);
            }
        }

        return hardware_interface::return_type::OK;
    }

    // std::vector<hardware_interface::StateInterface> contactSim::export_state_interfaces()
    // {
    //     std::vector<hardware_interface::StateInterface> state_interfaces;

    //     hardware_interface::InterfaceInfo info;
    //     info.name = "contact";
    //     info.data_type = "bool";

    //     for (size_t i = 0; i < 4; i++)
    //     {
    //         hardware_interface::InterfaceDescription desc(tipNames_[i],info);
    //         state_interfaces.emplace_back(hardware_interface::StateInterface(desc));
    //     }

    //     return state_interfaces;
    // }
} // namespace legged

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(legged::contactSim, hardware_interface::SensorInterface)