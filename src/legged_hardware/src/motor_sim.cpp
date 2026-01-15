#include "legged_hardware/motor_sim.hpp"

namespace legged
{
    hardware_interface::CallbackReturn MotorSim::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SystemInterface::on_init(hardware_interface::HardwareComponentInterfaceParams{params}) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        joint_state_topic_ = info_.hardware_parameters["joint_state_topic"];
        joint_command_topic_ = info_.hardware_parameters["joint_command_topic"];

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MotorSim::on_configure(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        auto cmd_pub = get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_command_topic_, 10);
        joint_command_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<trajectory_msgs::msg::JointTrajectory>>(cmd_pub);

        joint_state_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
            joint_state_topic_,
            1,
            [this](const sensor_msgs::msg::JointState::ConstSharedPtr &msg)
            {
                received_msg_.set(*msg);
            });

        joint_command_.joint_names.resize(12);
        joint_command_.points.resize(1);
        joint_command_.points[0].positions.resize(12);
        joint_command_.points[0].velocities.resize(12);
        joint_command_.points[0].effort.resize(12);
        reach_time_sec_ = 0;
        reach_time_nanosec_ = 0;

        joint_command_.joint_names.insert(joint_command_.joint_names.begin(), joint_names_, joint_names_ + 12);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MotorSim::on_activate(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MotorSim::on_deactivate(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type MotorSim::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        auto msg = received_msg_.try_get();
        if (msg == std::nullopt)
        {
            return hardware_interface::return_type::ERROR;
        }

        for (size_t i = 0; i < 12; i++)
        {
            set_state(joint_names_[i] + "/position", msg.value().position[i]);
            set_state(joint_names_[i] + "/velocity", msg.value().velocity[i]);
            set_state(joint_names_[i] + "/effort", msg.value().effort[i]);
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MotorSim::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        for (size_t i = 0; i < 12; i++)
        {
            command_position_[i] = get_command<double>(joint_names_[i] + "/position");
            command_velocity_[i] = get_command<double>(joint_names_[i] + "/velocity");
            command_effort_[i] = get_command<double>(joint_names_[i] + "/effort");

            joint_command_.points[0].positions[i] = command_position_[i];
            joint_command_.points[0].velocities[i] = command_velocity_[i];
            joint_command_.points[0].effort[i] = command_effort_[i];
        }
        joint_command_.points[0].time_from_start.sec = reach_time_sec_ + static_cast<int32_t>(period.seconds());
        joint_command_.points[0].time_from_start.nanosec = reach_time_nanosec_ + period.nanoseconds();

        if (joint_command_publisher_->trylock())
        {
            joint_command_publisher_->msg_ = joint_command_;
            joint_command_publisher_->unlockAndPublish();
        }

        return hardware_interface::return_type::OK;
    }

    // std::vector<hardware_interface::StateInterface> MotorSim::export_state_interfaces()
    // {
    //     std::vector<hardware_interface::StateInterface> state_interfaces;
    //     for (size_t i = 0; i < 12; i++)
    //     {
    //         state_interfaces.emplace_back(
    //             hardware_interface::StateInterface(joint_names_[i], "position", &current_position_[i]));
    //         state_interfaces.emplace_back(
    //             hardware_interface::StateInterface(joint_names_[i], "velocity", &current_velocity_[i]));
    //         state_interfaces.emplace_back(
    //             hardware_interface::StateInterface(joint_names_[i], "effort", &current_effort_[i]));
    //     }

    //     return state_interfaces;
    // }

    // std::vector<hardware_interface::CommandInterface> MotorSim::export_command_interfaces()
    // {
    //     std::vector<hardware_interface::CommandInterface> command_interfaces;

    //     for (size_t i = 0; i < 12; i++)
    //     {
    //         command_interfaces.emplace_back(
    //             hardware_interface::CommandInterface(joint_names_[i], "position", &command_position_[i]));
    //         command_interfaces.emplace_back(
    //             hardware_interface::CommandInterface(joint_names_[i], "velocity", &command_velocity_[i]));
    //         command_interfaces.emplace_back(
    //             hardware_interface::CommandInterface(joint_names_[i], "effort", &command_effort_[i]));
    //     }

    //     return command_interfaces;
    // }
} // namespace legged

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(legged::MotorSim, hardware_interface::SystemInterface)