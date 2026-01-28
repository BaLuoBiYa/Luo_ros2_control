#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace legged {
    class Target {
      public:
        Target(rclcpp::Node::SharedPtr &nh, const std::string &topicPrefix);

      private:
        std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;
        rclcpp::Node::SharedPtr nh_;
        rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
        tf2_ros::Buffer::SharedPtr buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_;
        realtime_tools::RealtimeThreadSafeBox<ocs2::SystemObservation> latestObservationBox_;

        ocs2::TargetTrajectories targetPoseToTargetTrajectories(const ocs2::vector_t &targetPose,
                                                                const ocs2::SystemObservation &observation,
                                                                const ocs2::scalar_t &targetReachingTime);
        ocs2::TargetTrajectories goalToTargetTrajectories(const ocs2::vector_t &goal,
                                                          const ocs2::SystemObservation &observation);
        ocs2::TargetTrajectories cmdVelToTargetTrajectories(const ocs2::vector_t &cmdVel,
                                                            const ocs2::SystemObservation &observation);
        ocs2::scalar_t estimateTimeToTarget(const ocs2::vector_t &desiredBaseDisplacement);
    };
} // namespace legged