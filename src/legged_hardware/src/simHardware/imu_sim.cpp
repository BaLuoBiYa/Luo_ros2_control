#include "legged_hardware/simHardware/imu_sim.hpp"

namespace legged {
    hardware_interface::CallbackReturn
    ImuSim::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
        if (hardware_interface::SystemInterface::on_init(hardware_interface::HardwareComponentInterfaceParams{
                params}) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        imuTopic_ = info_.hardware_parameters["imu_topic"];
        imuName_ = info_.hardware_parameters["imu_name"];

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    bool ImuSim::initSim(rclcpp::Node::SharedPtr &model_nh, std::map<std::string, sim::Entity> &joints,
                         const hardware_interface::HardwareInfo &hardware_info, sim::EntityComponentManager &_ecm,
                         unsigned int update_rate) {
        (void)model_nh;
        (void)joints;
        (void)hardware_info;
        (void)_ecm;
        (void)update_rate;
        return true;
    }

    hardware_interface::CallbackReturn ImuSim::on_configure(const rclcpp_lifecycle::State &pre) {
        (void)pre;
        node_.Subscribe(imuTopic_, &ImuSim::imuCallback, this);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ImuSim::on_activate(const rclcpp_lifecycle::State &pre) {
        (void)pre;
        gz::msgs::IMU emptyMsg;
        receivedImuMsg_.try_set(emptyMsg);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ImuSim::on_deactivate(const rclcpp_lifecycle::State &pre) {
        (void)pre;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ImuSim::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)time;
        (void)period;

        auto imu_msg = receivedImuMsg_.try_get();
        if (imu_msg == std::nullopt) {
            return hardware_interface::return_type::OK;
        }

        set_state<double>(imuName_ + "/orientation.x", imu_msg.value().orientation().x());
        set_state<double>(imuName_ + "/orientation.y", imu_msg.value().orientation().y());
        set_state<double>(imuName_ + "/orientation.z", imu_msg.value().orientation().z());
        set_state<double>(imuName_ + "/orientation.w", imu_msg.value().orientation().w());

        set_state<double>(imuName_ + "/angular_velocity.x", imu_msg.value().angular_velocity().x());
        set_state<double>(imuName_ + "/angular_velocity.y", imu_msg.value().angular_velocity().y());
        set_state<double>(imuName_ + "/angular_velocity.z", imu_msg.value().angular_velocity().z());

        set_state<double>(imuName_ + "/linear_acceleration.x", imu_msg.value().linear_acceleration().x());
        set_state<double>(imuName_ + "/linear_acceleration.y", imu_msg.value().linear_acceleration().y());
        set_state<double>(imuName_ + "/linear_acceleration.z", imu_msg.value().linear_acceleration().z());

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ImuSim::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)time;
        (void)period;
        return hardware_interface::return_type::OK;
    }

    void ImuSim::imuCallback(const gz::msgs::IMU &msg) {
        receivedImuMsg_.try_set(msg);
    }
} // namespace legged

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(legged::ImuSim, gz_ros2_control::GazeboSimSystemInterface)