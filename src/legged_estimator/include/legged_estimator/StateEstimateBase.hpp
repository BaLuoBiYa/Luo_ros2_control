//
// Created by qiayuan on 2021/11/15.
//
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <realtime_tools/realtime_publisher.h>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged_robot
{

	class StateEstimateBase
	{
	public:
		StateEstimateBase(ocs2::PinocchioInterface pinocchioInterface,
						  ocs2::CentroidalModelInfo info,
						  const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
						  const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

		virtual void updateJointStates(const ocs2::vector_t &jointPos, const ocs2::vector_t &jointVel);
		virtual void updateContact(ocs2::legged_robot::contact_flag_t contactFlag) { contactFlag_ = contactFlag; }
		virtual void updateImu(const Eigen::Quaternion<ocs2::scalar_t> &quat,
							   const ocs2::legged_robot::vector3_t &angularVelLocal,
							   const ocs2::legged_robot::vector3_t &linearAccelLocal,
							   const ocs2::legged_robot::matrix3_t &orientationCovariance,
							   const ocs2::legged_robot::matrix3_t &angularVelCovariance,
							   const ocs2::legged_robot::matrix3_t &linearAccelCovariance);

		virtual ocs2::vector_t update(const rclcpp::Time &time, const rclcpp::Duration &period) = 0;

		size_t getMode() { return ocs2::legged_robot::stanceLeg2ModeNumber(contactFlag_); }

	protected:
		void updateAngular(const ocs2::legged_robot::vector3_t &zyx, const ocs2::vector_t &angularVel);
		void updateLinear(const ocs2::vector_t &pos, const ocs2::vector_t &linearVel);
		void publishMsgs(const nav_msgs::msg::Odometry &odom);

		ocs2::PinocchioInterface pinocchioInterface_;
		ocs2::CentroidalModelInfo info_;
		std::unique_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematics_;

		ocs2::legged_robot::vector3_t zyxOffset_ = ocs2::legged_robot::vector3_t::Zero();
		ocs2::vector_t rbdState_;
		ocs2::legged_robot::contact_flag_t contactFlag_{};
		Eigen::Quaternion<ocs2::scalar_t> quat_;
		ocs2::legged_robot::vector3_t angularVelLocal_, linearAccelLocal_;
		ocs2::legged_robot::matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;

		std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odomPub_;
		std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>> posePub_;
		rclcpp::Time lastPub_;
	};

	template <typename T>
	T square(T a)
	{
		return a * a;
	}

	template <typename SCALAR_T>
	Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T> &q)
	{
		Eigen::Matrix<SCALAR_T, 3, 1> zyx;

		SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
		zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
		zyx(1) = std::asin(as);
		zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
		return zyx;
	}

} // namespace legged
