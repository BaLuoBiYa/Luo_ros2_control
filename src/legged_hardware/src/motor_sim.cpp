#include "legged_hardware/motor_sim.hpp"

namespace legged
{
    hardware_interface::CallbackReturn MotorSim::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SystemInterface::on_init(hardware_interface::HardwareComponentInterfaceParams{params}) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        jointStateTopic_ = info_.hardware_parameters["joint_state_topic"];
        jointCommandTopic_ = info_.hardware_parameters["joint_command_topic"];

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MotorSim::on_configure(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        auto cmd_pub = get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(jointCommandTopic_, 10);
        jointCommandPublisher_ = std::make_shared<realtime_tools::RealtimePublisher<trajectory_msgs::msg::JointTrajectory>>(cmd_pub);

        jointStateSubscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
            jointStateTopic_,
            1,
            [this](const sensor_msgs::msg::JointState::ConstSharedPtr &msg)
            {
                receivedMsg_.set(*msg);
            });

        jointCommand_.joint_names.resize(12);
        jointCommand_.points.resize(1);
        jointCommand_.points[0].positions.resize(12);
        jointCommand_.points[0].velocities.resize(12);
        jointCommand_.points[0].effort.resize(12);
        reachTimeSec_ = 0;
        reachTimeNanosec_ = 0;

        jointCommand_.joint_names.insert(jointCommand_.joint_names.begin(), jointNames_, jointNames_ + 12);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MotorSim::on_activate(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        sensor_msgs::msg::JointState empty_msg;
        receivedMsg_.set(empty_msg);
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
        auto msg = receivedMsg_.try_get();
        if (msg == std::nullopt)
        {
            return hardware_interface::return_type::ERROR;
        }

        for (size_t i = 0; i < 12; i++)
        {
            set_state<double>(jointNames_[i] + "/position", msg.value().position[i]);
            set_state<double>(jointNames_[i] + "/velocity", msg.value().velocity[i]);
            set_state<double>(jointNames_[i] + "/effort", msg.value().effort[i]);
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MotorSim::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        for (size_t i = 0; i < 12; i++)
        {
            commandPosition_[i] = get_command<double>(jointNames_[i] + "/position");
            commandVelocity_[i] = get_command<double>(jointNames_[i] + "/velocity");
            commandEffort_[i] = get_command<double>(jointNames_[i] + "/effort");

            jointCommand_.points[0].positions[i] = commandPosition_[i];
            jointCommand_.points[0].velocities[i] = commandVelocity_[i];
            jointCommand_.points[0].effort[i] = commandEffort_[i];
        }
        jointCommand_.points[0].time_from_start.sec = reachTimeSec_ + static_cast<int32_t>(period.seconds());
        jointCommand_.points[0].time_from_start.nanosec = reachTimeNanosec_ + period.nanoseconds();

        if (jointCommandPublisher_->trylock())
        {
            jointCommandPublisher_->msg_ = jointCommand_;
            jointCommandPublisher_->unlockAndPublish();
        }

        return hardware_interface::return_type::OK;
    }

    // std::vector<hardware_interface::StateInterface> MotorSim::export_state_interfaces()
    // {
    //     std::vector<hardware_interface::StateInterface> state_interfaces;
    //     for (size_t i = 0; i < 12; i++)
    //     {
    //         state_interfaces.emplace_back(
    //             hardware_interface::StateInterface(jointNames_[i], "position", &current_position_[i]));
    //         state_interfaces.emplace_back(
    //             hardware_interface::StateInterface(jointNames_[i], "velocity", &current_velocity_[i]));
    //         state_interfaces.emplace_back(
    //             hardware_interface::StateInterface(jointNames_[i], "effort", &current_effort_[i]));
    //     }

    //     return state_interfaces;
    // }

    // std::vector<hardware_interface::CommandInterface> MotorSim::export_command_interfaces()
    // {
    //     std::vector<hardware_interface::CommandInterface> command_interfaces;

    //     for (size_t i = 0; i < 12; i++)
    //     {
    //         command_interfaces.emplace_back(
    //             hardware_interface::CommandInterface(jointNames_[i], "position", &commandPosition_[i]));
    //         command_interfaces.emplace_back(
    //             hardware_interface::CommandInterface(jointNames_[i], "velocity", &command_velocity_[i]));
    //         command_interfaces.emplace_back(
    //             hardware_interface::CommandInterface(jointNames_[i], "effort", &command_effort_[i]));
    //     }

    //     return command_interfaces;
    // }
} // namespace legged

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(legged::MotorSim, hardware_interface::SystemInterface)