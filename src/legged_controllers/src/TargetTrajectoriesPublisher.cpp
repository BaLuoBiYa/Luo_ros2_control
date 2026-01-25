//
// Created by qiayuan on 2022/7/24.
//

#include "legged_controllers/TargetTrajectoriesPublisher.hpp"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace legged {
    TargetTrajectoriesPublisher::TargetTrajectoriesPublisher(const rclcpp::Node::SharedPtr &nh,
                                                             const std::string &topicPrefix)
        : nh_(nh) {
        buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
        tf2_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, nh_, true);

        //Trajectories publisher
        targetTrajectoriesPublisher_.reset(new ocs2::TargetTrajectoriesRosPublisher(nh_, topicPrefix));

        // observation subscriber
        observationSub_ = nh_->create_subscription<ocs2_msgs::msg::MpcObservation>(
                topicPrefix + "_mpc_observation", 1,
                std::bind(&TargetTrajectoriesPublisher::observationCallback, this, std::placeholders::_1));
        goalSub_ =
            nh->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1,
                std::bind(&TargetTrajectoriesPublisher::goalCallback, this, std::placeholders::_1));
        cmdVelSub_ = 
            nh->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, 
                std::bind(&TargetTrajectoriesPublisher::cmdVelCallback, this, std::placeholders::_1));
    }

    void TargetTrajectoriesPublisher::observationCallback(const ocs2_msgs::msg::MpcObservation::ConstSharedPtr &msg) {
        std::lock_guard<std::mutex> lock(latestObservationMutex_);
        latestObservation_ = ocs2::ros_msg_conversions::readObservationMsg(*msg);
    }

    void TargetTrajectoriesPublisher::goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
        if (latestObservation_.time == 0.0) {
            return;
        }
        geometry_msgs::msg::PoseStamped pose = *msg;
        try {
            buffer_->transform(pose, pose, "odom", tf2::durationFromSec(0.2));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->nh_->get_logger(), "Failure %s\n", ex.what());
            return;
        }

        ocs2::vector_t cmdGoal;
        cmdGoal.setZero(6);
        cmdGoal[0] = pose.pose.position.x;
        cmdGoal[1] = pose.pose.position.y;
        cmdGoal[2] = pose.pose.position.z;
        Eigen::Quaternion<ocs2::scalar_t> q(
            pose.pose.orientation.w,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z);
        // cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
        // cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
        // cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

    //     // const auto trajectories = goalToTargetTrajectories_(cmdGoal, latestObservation_);
    //     targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    }

    // void TargetTrajectoriesPublisher::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &msg) {
    //     if (latestObservation_.time == 0.0) {
    //         return;
    //     }

    //     vector_t cmdVel = vector_t::Zero(4);
    //     cmdVel[0] = msg->linear.x;
    //     cmdVel[1] = msg->linear.y;
    //     cmdVel[2] = msg->linear.z;
    //     cmdVel[3] = msg->angular.z;

    //     // const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, latestObservation_);
    //     targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    // }
}



















































































//     ocs2::scalar_t estimateTimeToTarget(const ocs2::vector_t &desiredBaseDisplacement) {
//         const ocs2::scalar_t &dx = desiredBaseDisplacement(0);
//         const ocs2::scalar_t &dy = desiredBaseDisplacement(1);
//         const ocs2::scalar_t &dyaw = desiredBaseDisplacement(3);
//         const ocs2::scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
//         const ocs2::scalar_t displacement = std::sqrt(dx * dx + dy * dy);
//         const ocs2::scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
//         return std::max(rotationTime, displacementTime);
//     }

//     ocs2::TargetTrajectories targetPoseToTargetTrajectories(const ocs2::vector_t &targetPose,
//                                                             const ocs2::SystemObservation &observation,
//                                                             const ocs2::scalar_t &targetReachingTime) {
//         // desired time trajectory
//         const ocs2::scalar_array_t timeTrajectory{observation.time, targetReachingTime};

//         // desired state trajectory
//         ocs2::vector_t currentPose = observation.state.segment<6>(6);
//         currentPose(2) = COM_HEIGHT;
//         currentPose(4) = 0;
//         currentPose(5) = 0;
//         ocs2::vector_array_t stateTrajectory(2, ocs2::vector_t::Zero(observation.state.size()));
//         stateTrajectory[0] << ocs2::vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
//         stateTrajectory[1] << ocs2::vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

//         // desired input trajectory (just right dimensions, they are not used)
//         const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

//         return {timeTrajectory, stateTrajectory, inputTrajectory};
//     }

//     ocs2::TargetTrajectories goalToTargetTrajectories(const ocs2::vector_t &goal,
//                                                       const ocs2::SystemObservation &observation) {
//         const ocs2::vector_t currentPose = observation.state.segment<6>(6);
//         const ocs2::vector_t targetPose = [&]() {
//             ocs2::vector_t target(6);
//             target(0) = goal(0);
//             target(1) = goal(1);
//             target(2) = ocs2::COM_HEIGHT;
//             target(3) = goal(3);
//             target(4) = 0;
//             target(5) = 0;
//             return target;
//         }();
//         const ocs2::scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
//         return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
//     }

//     ocs2::TargetTrajectories cmdVelToTargetTrajectories(const ocs2::vector_t &cmdVel,
//                                                         const ocs2::SystemObservation &observation) {
//         const ocs2::vector_t currentPose = observation.state.segment<6>(6);
//         const Eigen::Matrix<ocs2::scalar_t, 3, 1> zyx = currentPose.tail(3);
//         ocs2::vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);

//         const ocs2::scalar_t timeToTarget = TIME_TO_TARGET;
//         const ocs2::vector_t targetPose = [&]() {
//             ocs2::vector_t target(6);
//             target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
//             target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
//             target(2) = COM_HEIGHT;
//             target(3) = currentPose(3) + cmdVel(3) * timeToTarget;
//             target(4) = 0;
//             target(5) = 0;
//             return target;
//         }();

//         // target reaching duration
//         const ocs2::scalar_t targetReachingTime = observation.time + timeToTarget;
//         auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
//         trajectories.stateTrajectory[0].head(3) = cmdVelRot;
//         trajectories.stateTrajectory[1].head(3) = cmdVelRot;
//         return trajectories;
//     }

// } // namespace legged

// int main(int argc, char **argv) {
//     const std::string robotName = "legged_robot";

//     // Initialize ros node
//     rclcpp::init(argc, argv);
//     rclcpp::Node::SharedPtr nodeHandle = rclcpp::Node::make_shared(robotName + "_target");

//     // Get node parameters
//     std::string referenceFile;
//     std::string taskFile;
//     nodeHandle->declare_parameter<std::string>("/referenceFile", referenceFile);
//     nodeHandle->declare_parameter<std::string>("/taskFile", taskFile);

//     loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);
//     loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
//     loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
//     loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
//     loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

//     // TargetTrajectoriesPublisher target_pose_command(nodeHandle, robotName, &goalToTargetTrajectories,
//     //                                                 &cmdVelToTargetTrajectories);

//     // Successful exit
//     return 0;
// }
