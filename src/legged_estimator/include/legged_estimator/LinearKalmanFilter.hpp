//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "legged_estimator/StateEstimateBase.hpp"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_core/misc/LoadData.h>

#include <realtime_tools/realtime_buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace legged_robot
{

	class KalmanFilterEstimate : public StateEstimateBase
	{
	public:
		KalmanFilterEstimate(ocs2::PinocchioInterface pinocchioInterface, 
							 ocs2::CentroidalModelInfo info, 
							 const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
							 const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

		ocs2::vector_t update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

		void loadSettings(const std::string &taskFile, bool verbose);

	protected:
		void updateFromTopic();

		void callback(const nav_msgs::msg::Odometry::ConstPtr &msg);

		nav_msgs::msg::Odometry getOdomMsg();

		ocs2::vector_t feetHeights_;
		// Config
		ocs2::scalar_t footRadius_ = 0.02;
		ocs2::scalar_t imuProcessNoisePosition_ = 0.02;
		ocs2::scalar_t imuProcessNoiseVelocity_ = 0.02;
		ocs2::scalar_t footProcessNoisePosition_ = 0.002;
		ocs2::scalar_t footSensorNoisePosition_ = 0.005;
		ocs2::scalar_t footSensorNoiseVelocity_ = 0.1;
		ocs2::scalar_t footHeightSensorNoise_ = 0.01;

	private:
		size_t numContacts_, dimContacts_, numState_, numObserve_;

		ocs2::matrix_t a_, b_, c_, q_, p_, r_;
		ocs2::vector_t xHat_, ps_, vs_;

		// Topic
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
		realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> buffer_;
		std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
		std::shared_ptr<tf2_ros::TransformListener> tfListener_;
		tf2::Transform world2odom_;
		std::string frameOdom_, frameGuess_;
		bool topicUpdated_;
	};

} // namespace legged_robot