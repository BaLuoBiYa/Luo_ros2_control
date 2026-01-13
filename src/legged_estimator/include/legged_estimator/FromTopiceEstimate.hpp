//
// Created by qiayuan on 2022/7/24.
//

#include "legged_estimator/StateEstimateBase.hpp"

#include <realtime_tools/realtime_buffer.hpp>

#pragma once
namespace legged
{

	class FromTopicStateEstimate : public StateEstimateBase
	{
	public:
		FromTopicStateEstimate(ocs2::PinocchioInterface pinocchioInterface, 
							   ocs2::CentroidalModelInfo info,
							   const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
							   const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

		void updateImu(const Eigen::Quaternion<ocs2::scalar_t> &quat, const ocs2::legged_robot::vector3_t &angularVelLocal, const ocs2::legged_robot::vector3_t &linearAccelLocal,
					const ocs2::legged_robot::matrix3_t &orientationCovariance, const ocs2::legged_robot::matrix3_t &angularVelCovariance,
					const ocs2::legged_robot::matrix3_t &linearAccelCovariance) override {};

		ocs2::vector_t update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

	private:
		void callback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);

		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
		realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> buffer_;
	};

} // namespace legged
