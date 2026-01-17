#include "legged_hardware/imu_sim.hpp"

namespace legged
{
    hardware_interface::CallbackReturn ImuSim::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SensorInterface::on_init(hardware_interface::HardwareComponentInterfaceParams{params}) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        imuTopic_ = info_.hardware_parameters["imu_topic"];
        imuName_ = info_.hardware_parameters["imu_name"];

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ImuSim::on_configure(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        imuSubscriber_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
            imuTopic_,
            1,
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
            {
                receivedImuMsg_.try_set(*msg);
            });

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ImuSim::on_activate(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        sensor_msgs::msg::Imu empty_msg;
        receivedImuMsg_.set(empty_msg);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ImuSim::on_deactivate(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ImuSim::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;

        auto imu_msg = receivedImuMsg_.try_get();
        if (imu_msg == std::nullopt)
        {
            return hardware_interface::return_type::ERROR;
        }

        set_state<double>("imu/orientation.x", imu_msg.value().orientation.x);
        set_state<double>("imu/orientation.y", imu_msg.value().orientation.y);
        set_state<double>("imu/orientation.z", imu_msg.value().orientation.z);
        set_state<double>("imu/orientation.w", imu_msg.value().orientation.w);

        set_state<double>("imu/angular_velocity.x", imu_msg.value().angular_velocity.x);
        set_state<double>("imu/angular_velocity.y", imu_msg.value().angular_velocity.y);
        set_state<double>("imu/angular_velocity.z", imu_msg.value().angular_velocity.z);

        set_state<double>("imu/linear_acceleration.x", imu_msg.value().linear_acceleration.x);
        set_state<double>("imu/linear_acceleration.y", imu_msg.value().linear_acceleration.y);
        set_state<double>("imu/linear_acceleration.z", imu_msg.value().linear_acceleration.z);

        return hardware_interface::return_type::OK;
    }

    // std::vector<hardware_interface::StateInterface> ImuSim::export_state_interfaces()
    // {
    //     std::vector<hardware_interface::StateInterface> state_interfaces;

    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         imuName_, "orientation.x", &orientation_[0]));
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         imuName_, "orientation.y", &orientation_[1]));
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         imuName_, "orientation.z", &orientation_[2]));
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         imuName_, "orientation.w", &orientation_[3]));

    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         imuName_, "angular_velocity.x", &angular_velocity_[0]));
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         imuName_, "angular_velocity.y", &angular_velocity_[1]));
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         imuName_, "angular_velocity.z", &angular_velocity_[2]));

    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         imuName_, "linear_acceleration.x", &linear_acceleration_[0]));
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         imuName_, "linear_acceleration.y", &linear_acceleration_[1]));
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         imuName_, "linear_acceleration.z", &linear_acceleration_[2]));

    //     return state_interfaces;
    // }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(legged::ImuSim, hardware_interface::SensorInterface)