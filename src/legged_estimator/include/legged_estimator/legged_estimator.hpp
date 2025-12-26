#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <iomanip>

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> //#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> doesn't work on Ubuntu20.04
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sensor_msgs/msg/imu.hpp>

#include "fusion_estimator/msg/fusion_estimator_test.hpp"
#include "GO2FusionEstimator/SensorBase.h"
#include "GO2FusionEstimator/Sensor_Legs.h"
#include "GO2FusionEstimator/Sensor_IMU.h"

namespace legged_controllers{
    class legged_estimator:public controller_interface::ControllerInterface{
        protected:

        public:
        legged_estimator() = default;
        ~legged_estimator();

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    };//class legged_estimator
}// namespace legged_controllers