#include "legged_estimator/StateEstimateBase.hpp"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace legged {

    StateEstimateBase::StateEstimateBase(ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo info,
                                         const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
                                         const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : pinocchioInterface_(std::move(pinocchioInterface)), info_(std::move(info)),
          eeKinematics_(eeKinematics.clone()), rbdState_(ocs2::vector_t ::Zero(2 * info_.generalizedCoordinatesNum)) {
        auto odom = node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        odomPub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom);
        auto pose = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);
        posePub_ =
            std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>>(pose);
    }

    void StateEstimateBase::updateJointStates(const ocs2::vector_t &jointPos, const ocs2::vector_t &jointVel) {
        rbdState_.segment(6, info_.actuatedDofNum) = jointPos;
        rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = jointVel;
    }

    void StateEstimateBase::updateImu(const Eigen::Quaternion<ocs2::scalar_t> &quat,
                                      const ocs2::legged_robot::vector3_t &angularVelLocal,
                                      const ocs2::legged_robot::vector3_t &linearAccelLocal,
                                      const ocs2::legged_robot::matrix3_t &orientationCovariance,
                                      const ocs2::legged_robot::matrix3_t &angularVelCovariance,
                                      const ocs2::legged_robot::matrix3_t &linearAccelCovariance) {
        quat_ = quat;
        angularVelLocal_ = angularVelLocal;
        linearAccelLocal_ = linearAccelLocal;
        orientationCovariance_ = orientationCovariance;
        angularVelCovariance_ = angularVelCovariance;
        linearAccelCovariance_ = linearAccelCovariance;

        ocs2::legged_robot::vector3_t zyx = quatToZyx(quat) - zyxOffset_;
        ocs2::legged_robot::vector3_t angularVelGlobal =
            ocs2::getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<ocs2::scalar_t>(
                zyx, ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity<ocs2::scalar_t>(quatToZyx(quat),
                                                                                                angularVelLocal));
        updateAngular(zyx, angularVelGlobal);
    }

    void StateEstimateBase::updateImu(const Eigen::Quaternion<ocs2::scalar_t> &quat,
                                      const ocs2::legged_robot::vector3_t &angularVelLocal,
                                      const ocs2::legged_robot::vector3_t &linearAccelLocal) {
        quat_ = quat;
        angularVelLocal_ = angularVelLocal;
        linearAccelLocal_ = linearAccelLocal;

        ocs2::legged_robot::vector3_t zyx = quatToZyx(quat) - zyxOffset_;
        ocs2::legged_robot::vector3_t angularVelGlobal =
            ocs2::getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<ocs2::scalar_t>(
                zyx, ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity<ocs2::scalar_t>(quatToZyx(quat),
                                                                                                angularVelLocal));
        updateAngular(zyx, angularVelGlobal);
    }

    void StateEstimateBase::updateAngular(const ocs2::legged_robot::vector3_t &zyx, const ocs2::vector_t &angularVel) {
        rbdState_.segment<3>(0) = zyx;
        rbdState_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
    }

    void StateEstimateBase::updateLinear(const ocs2::vector_t &pos, const ocs2::vector_t &linearVel) {
        rbdState_.segment<3>(3) = pos;
        rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
    }

    void StateEstimateBase::publishMsgs(const nav_msgs::msg::Odometry &odom) {
        if (odomPub_->trylock()) {
            odomPub_->msg_ = odom;
            odomPub_->unlockAndPublish();
        }
        if (posePub_->trylock()) {
            posePub_->msg_.header = odom.header;
            posePub_->msg_.pose = odom.pose;
            posePub_->unlockAndPublish();
        }
    }

} // namespace legged
