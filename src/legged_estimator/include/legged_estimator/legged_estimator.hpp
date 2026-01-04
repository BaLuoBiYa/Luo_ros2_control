#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <iomanip>

#include <controller_interface/chainable_controller_interface.hpp>
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

namespace legged_robot{
    class legged_estimator:public controller_interface::ChainableControllerInterface{
        protected:
            union imu_msg_t{
                double data[10];
                struct {
                    struct orientation{
                        double x;
                        double y;
                        double z;
                        double w;
                    }orientation;
                    struct linear_acceleration{
                        double x;
                        double y;
                        double z;
                    }linear_acceleration;
                    struct angular_velocity{
                        double x;
                        double y;
                        double z;
                    }angular_velocity;
                };
            };

            std::vector<EstimatorPortN*> StateSpaceModel_Go2_Sensors;
            std::vector<std::string> joint_names_;
            std::vector<std::string> tipforce_names_;
            std::string imu_name_;

            std::shared_ptr<DataFusion::SensorIMUAcc>     Sensor_IMUAcc;
            std::shared_ptr<DataFusion::SensorIMUMagGyro> Sensor_IMUMagGyro;
            std::shared_ptr<DataFusion::SensorLegsPos>    Sensor_LegsPos;
            std::shared_ptr<DataFusion::SensorLegsOri>    Sensor_LegsOri;

            fusion_estimator::msg::FusionEstimatorTest fusion_msg;
            imu_msg_t imu_msg_;
            std::vector<double> joint_msg_;
            double contact_state_;

            rclcpp::Publisher<fusion_estimator::msg::FusionEstimatorTest>::SharedPtr FETest_publisher;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr SMXFE_publisher, SMXFE_2D_publisher;

            std::string pub_odom_topic,pub_odom_2d_topic,pub_estimation_topic;
            std::string odom_frame_id, child_frame_id, child_frame_2d_id;
            bool imu_data_enable, leg_pos_enable, leg_ori_enable;
            bool publish_odom_enable;
            double leg_ori_init_weight, leg_ori_time_wight;
            double position_correct[9];
            double orientation_correct[9];
            double CurrentTimestamp;

            double LastIMU[3][100]={0};
            double LastJoint[3][100]={0};

            void imu_update(const imu_msg_t msg);
            void joint_update(const std::vector<double> arr);
            void publish_estimation();

        public:

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        // controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;
        // std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
        // bool on_set_chained_mode(bool chained_mode) override;

        controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        controller_interface::return_type update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    };//class legged_estimator
}// namespace legged_controllers