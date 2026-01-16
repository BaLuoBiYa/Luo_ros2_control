#include "legged_hardware/contact_sim.hpp"

namespace legged
{
    hardware_interface::CallbackReturn contactSim::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SensorInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        contacts_topic_ = info_.hardware_parameters["contacts_topic"];

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn contactSim::on_configure(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        for (int i = 0; i < 4; i++)
        {
            contact_flag_[i] = false;
        }

        for (size_t i = 0; i < 4; i++)
        {
            std::string topic = contacts_topic_ + tip_names_[i];
            contact_subscriber_[i] = get_node()->create_subscription<ros_gz_interfaces::msg::Contacts>(
                topic,
                1,
                [this, i](const ros_gz_interfaces::msg::Contacts::ConstSharedPtr &msg)
                {
                    this->received_contacts_msg_[i].set(*msg);
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
            received_contacts_msg_[i].set(empty_msg);
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
            auto msg = received_contacts_msg_[i].try_get();
            if (msg == std::nullopt)
            {
                return hardware_interface::return_type::ERROR;
            }
            bool hit = false;
            for (const auto &c : msg.value().contacts)
            {
                // 只要任一碰撞体为障碍物即可判定接触
                if (c.collision1.name == kObstacle || c.collision2.name == kObstacle)
                {
                    hit = true;
                    break;
                }
            }
            set_state<bool>(tip_names_[i] + "/contact", hit);
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
    //         hardware_interface::InterfaceDescription desc(tip_names_[i],info);
    //         state_interfaces.emplace_back(hardware_interface::StateInterface(desc));
    //     }

    //     return state_interfaces;
    // }
} // namespace legged

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(legged::contactSim, hardware_interface::SensorInterface)