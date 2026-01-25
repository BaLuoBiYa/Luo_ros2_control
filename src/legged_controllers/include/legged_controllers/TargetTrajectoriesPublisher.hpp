//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/subscription.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace legged {
    class TargetTrajectoriesPublisher final {
      public:
        TargetTrajectoriesPublisher(const rclcpp::Node::SharedPtr &nh, const std::string &topicPrefix);
        ocs2::scalar_t TARGET_DISPLACEMENT_VELOCITY;
        ocs2::scalar_t TARGET_ROTATION_VELOCITY;
        ocs2::scalar_t COM_HEIGHT;
        ocs2::vector_t DEFAULT_JOINT_STATE;
        ocs2::scalar_t TIME_TO_TARGET;

      private:
        const rclcpp::Node::SharedPtr nh_;
        std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

        rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
        tf2_ros::Buffer::SharedPtr buffer_;

        std::unique_ptr<tf2_ros::TransformListener> tf2_;

        mutable std::mutex latestObservationMutex_;
        ocs2::SystemObservation latestObservation_;

        void observationCallback(const ocs2_msgs::msg::MpcObservation::ConstSharedPtr &msg);
        void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
        void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg);
    };

} // namespace legged
