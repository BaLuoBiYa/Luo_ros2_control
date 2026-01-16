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
                    this->receivedContactsMsg_[i].set(*msg);
                });
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn contactSim::on_activate(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        ros_gz_interfaces::msg::Contacts empty_msg;
        for (size_t i = 0; i < 4; i++)
        {
            receivedContactsMsg_[i].set(empty_msg);
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
        (void)time;
        (void)period;
        const std::string kObstacle = "Obstacle::link::collision";

        for (size_t i = 0; i < 4; i++)
        {
            auto msg = receivedContactsMsg_[i].try_get();
            if (msg == std::nullopt)
            {
                return hardware_interface::return_type::ERROR;
            }

            rclcpp::Time msg_time(msg.value().header.stamp, get_node()->get_clock()->get_clock_type());
            double age = time.seconds() - msg_time.seconds();

            if (age > 0.01)
            {
                set_state<bool>("contact/" + tipNames_[i], false);
            }
            else
            {
                set_state<bool>("contact/" + tipNames_[i], true);
            }
            // bool hit = false;
            // for (const auto &c : msg.value().contacts)
            // {
            //     // 只要任一碰撞体为障碍物即可判定接触
            //     if (c.collision1.name == kObstacle || c.collision2.name == kObstacle)
            //     {
            //         hit = true;
            //         break;
            //     }
            // }
            // set_state<bool>(tipNames_[i] + "/contact", hit);
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