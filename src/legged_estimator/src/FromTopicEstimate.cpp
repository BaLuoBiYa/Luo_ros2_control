#include "legged_estimator/FromTopiceEstimate.hpp"

namespace legged
{

	FromTopicStateEstimate::FromTopicStateEstimate(ocs2::PinocchioInterface pinocchioInterface,
												   ocs2::CentroidalModelInfo info,
												   const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
												   const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
		: StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics, node)
	{
		sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
				"/ground_truth/state", 10,
				std::bind(&FromTopicStateEstimate::callback, this, std::placeholders::_1));
	}

	void FromTopicStateEstimate::callback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
	{
		buffer_.writeFromNonRT(*msg);
	}

	ocs2::vector_t FromTopicStateEstimate::update(const double & /*time*/, const double & /*period*/)
	{
		nav_msgs::msg::Odometry odom = *buffer_.readFromRT();

		updateAngular(quatToZyx(Eigen::Quaternion<ocs2::scalar_t>(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
																  odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)),
					  Eigen::Matrix<ocs2::scalar_t, 3, 1>(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z));
		updateLinear(Eigen::Matrix<ocs2::scalar_t, 3, 1>(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
					 Eigen::Matrix<ocs2::scalar_t, 3, 1>(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z));

		publishMsgs(odom);

		return rbdState_;
	}

} // namespace legged
