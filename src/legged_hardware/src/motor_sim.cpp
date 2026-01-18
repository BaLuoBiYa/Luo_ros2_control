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
        jointCommand_.points[0].positions.resize(12, 0.0);
        jointCommand_.points[0].velocities.resize(12, 0.0);
        jointCommand_.points[0].effort.resize(12, 0.0);
        jointCommand_.points[0].accelerations.resize(12, 0.0);
        reachTimeSec_ = 0;
        reachTimeNanosec_ = 0;
        
        for (size_t i = 0; i < 12; i++)
        {
            currentPosition_[i] = 0.0;
            currentVelocity_[i] = 0.0;
            currentEffort_[i] = 0.0;
            jointCommand_.joint_names[i] = jointNames_[i];
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MotorSim::on_activate(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        sensor_msgs::msg::JointState empty_msg;
        empty_msg.position.resize(12, 0.0);
        empty_msg.velocity.resize(12, 0.0);
        empty_msg.effort.resize(12, 0.0);
        receivedMsg_.try_set(empty_msg);

        for (size_t i = 0; i < 12; i++)
        {
            jointCommand_.points[0].positions[i] = 0.0;
            jointCommand_.points[0].velocities[i] = 0.0;
            jointCommand_.points[0].effort[i] = 0.0;
        }
        jointCommand_.points[0].time_from_start.sec = 0;
        jointCommand_.points[0].time_from_start.nanosec = 0;

        if (jointCommandPublisher_->trylock())
        {
            jointCommandPublisher_->msg_ = jointCommand_;
            jointCommandPublisher_->unlockAndPublish();
        }

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
            return hardware_interface::return_type::OK;
        }

        for (size_t i = 0; i < 12; i++)
        {
            set_state<double>(jointNames_[i] + "/position", msg.value().position[i]);
            set_state<double>(jointNames_[i] + "/velocity", msg.value().velocity[i]);
            // set_state<double>(jointNames_[i] + "/effort", msg.value().effort[i]);
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MotorSim::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        for (size_t i = 0; i < 12; i++)
        {
            for (size_t i = 0; i < 12; i++)
        {
            double pos = get_command<double>(jointNames_[i] + "/position");
            double vel = get_command<double>(jointNames_[i] + "/velocity");
            double eff = get_command<double>(jointNames_[i] + "/effort");
            if (!std::isfinite(pos)) pos = 0.0;
            if (!std::isfinite(vel)) vel = 0.0;
            if (!std::isfinite(eff)) eff = 0.0;
            jointCommand_.points[0].positions[i] = pos;
            jointCommand_.points[0].velocities[i] = vel;
            jointCommand_.points[0].effort[i] = eff;
        }
        }
        jointCommand_.points[0].time_from_start.sec += static_cast<int32_t>(period.seconds());

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