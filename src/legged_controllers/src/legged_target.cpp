#include "legged_controllers/legged_target.hpp"

namespace legged {
    ocs2::scalar_t TARGET_DISPLACEMENT_VELOCITY;
    ocs2::scalar_t TARGET_ROTATION_VELOCITY;
    ocs2::scalar_t COM_HEIGHT;
    ocs2::vector_t DEFAULT_JOINT_STATE(12);
    ocs2::scalar_t TIME_TO_TARGET;

    Target::Target(rclcpp::Node::SharedPtr &nh, const std::string &topicPrefix) {
        nh_ = nh;
        buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
        tf2_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, nh_, true);

        targetTrajectoriesPublisher_.reset(new ocs2::TargetTrajectoriesRosPublisher(nh, topicPrefix));

        // observation subscriber
        auto observationCallback = [this](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr &msg) {
            latestObservationBox_.try_set(ocs2::ros_msg_conversions::readObservationMsg(*msg));
        };
        observationSub_ = nh_->create_subscription<ocs2_msgs::msg::MpcObservation>(topicPrefix + "_mpc_observation",
                                                                                   rclcpp::QoS(1), observationCallback);

        // goal subscriber
        auto goalCallback = [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
            ocs2::SystemObservation latestObservation = latestObservationBox_.try_get().value();
            if (latestObservation.time == 0.0) {
                return;
            }

            geometry_msgs::msg::PoseStamped pose = *msg;
            try {
                buffer_->transform(pose, pose, "odom", tf2::durationFromSec(0.2));
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(nh_->get_logger(), "Failure %s\n", ex.what());
                return;
            }

            ocs2::vector_t cmdGoal = ocs2::vector_t::Zero(6);
            cmdGoal[0] = pose.pose.position.x;
            cmdGoal[1] = pose.pose.position.y;
            cmdGoal[2] = pose.pose.position.z;
            Eigen::Quaternion<ocs2::scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x,
                                                pose.pose.orientation.y, pose.pose.orientation.z);
            cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
            cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
            cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

            const ocs2::TargetTrajectories trajectories = this->goalToTargetTrajectories(cmdGoal, latestObservation);
            targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
        };
        goalSub_ = nh->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", rclcpp::QoS(1),
                                                                            goalCallback);

        // cmd_vel subscriber
        auto cmdVelCallback = [this](const geometry_msgs::msg::Twist::ConstSharedPtr &msg) {
            ocs2::SystemObservation latestObservation = latestObservationBox_.try_get().value();

            if (latestObservation.time == 0.0) {
                return;
            }

            ocs2::vector_t cmdVel = ocs2::vector_t::Zero(4);
            cmdVel[0] = msg->linear.x;
            cmdVel[1] = msg->linear.y;
            cmdVel[2] = msg->linear.z;
            cmdVel[3] = msg->angular.z;

            const ocs2::TargetTrajectories trajectories = cmdVelToTargetTrajectories(cmdVel, latestObservation);
            targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
        };
        cmdVelSub_ = nh->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(1), cmdVelCallback);
    }

    ocs2::scalar_t Target::estimateTimeToTarget(const ocs2::vector_t &desiredBaseDisplacement) {
        const ocs2::scalar_t &dx = desiredBaseDisplacement(0);
        const ocs2::scalar_t &dy = desiredBaseDisplacement(1);
        const ocs2::scalar_t &dyaw = desiredBaseDisplacement(3);
        const ocs2::scalar_t rotationTime = std::abs(dyaw) / legged::TARGET_ROTATION_VELOCITY;
        const ocs2::scalar_t displacement = std::sqrt(dx * dx + dy * dy);
        const ocs2::scalar_t displacementTime = displacement / legged::TARGET_DISPLACEMENT_VELOCITY;
        return std::max(rotationTime, displacementTime);
    }

    ocs2::TargetTrajectories Target::targetPoseToTargetTrajectories(const ocs2::vector_t &targetPose,
                                                                    const ocs2::SystemObservation &observation,
                                                                    const ocs2::scalar_t &targetReachingTime) {
        // desired time trajectory
        const ocs2::scalar_array_t timeTrajectory{observation.time, targetReachingTime};

        // desired state trajectory
        ocs2::vector_t currentPose = observation.state.segment<6>(6);
        currentPose(2) = legged::COM_HEIGHT;
        currentPose(4) = 0;
        currentPose(5) = 0;
        ocs2::vector_array_t stateTrajectory(2, ocs2::vector_t::Zero(observation.state.size()));
        stateTrajectory[0] << ocs2::vector_t::Zero(6), currentPose, legged::DEFAULT_JOINT_STATE;
        stateTrajectory[1] << ocs2::vector_t::Zero(6), targetPose, legged::DEFAULT_JOINT_STATE;

        // desired input trajectory (just right dimensions, they are not used)
        const ocs2::vector_array_t inputTrajectory(2, ocs2::vector_t::Zero(observation.input.size()));

        return {timeTrajectory, stateTrajectory, inputTrajectory};
    }

    ocs2::TargetTrajectories Target::goalToTargetTrajectories(const ocs2::vector_t &goal,
                                                              const ocs2::SystemObservation &observation) {
        const ocs2::vector_t currentPose = observation.state.segment<6>(6);
        const ocs2::vector_t targetPose = [&]() {
            ocs2::vector_t target(6);
            target(0) = goal(0);
            target(1) = goal(1);
            target(2) = legged::COM_HEIGHT;
            target(3) = goal(3);
            target(4) = 0;
            target(5) = 0;
            return target;
        }();
        const ocs2::scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
        return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
    }

    ocs2::TargetTrajectories Target::cmdVelToTargetTrajectories(const ocs2::vector_t &cmdVel,
                                                                const ocs2::SystemObservation &observation) {
        const ocs2::vector_t currentPose = observation.state.segment<6>(6);
        const Eigen::Matrix<ocs2::scalar_t, 3, 1> zyx = currentPose.tail(3);
        ocs2::vector_t cmdVelRot = ocs2::getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);

        const ocs2::scalar_t timeToTarget = legged::TIME_TO_TARGET;
        const ocs2::vector_t targetPose = [&]() {
            ocs2::vector_t target(6);
            target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
            target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
            target(2) = legged::COM_HEIGHT;
            target(3) = currentPose(3) + cmdVel(3) * timeToTarget;
            target(4) = 0;
            target(5) = 0;
            return target;
        }();

        // target reaching duration
        const ocs2::scalar_t targetReachingTime = observation.time + timeToTarget;
        auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
        trajectories.stateTrajectory[0].head(3) = cmdVelRot;
        trajectories.stateTrajectory[1].head(3) = cmdVelRot;
        return trajectories;
    }
} // namespace legged

int main(int argc, char *argv[]) {
    const std::string robotName = "legged_robot";

    // Initialize ros node
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(robotName + "_target");

    const std::string referenceFile = node->declare_parameter<std::string>("referenceFile", "");
    const std::string taskFile = node->declare_parameter<std::string>("taskFile", "");
    if (referenceFile.empty() || taskFile.empty()) {
        throw std::runtime_error("[LeggedRobotPoseCommandNode] Parameter 'referenceFile/taskFile' is required.");
    }

    ocs2::loadData::loadCppDataType(referenceFile, "comHeight", legged::COM_HEIGHT);
    ocs2::loadData::loadEigenMatrix(referenceFile, "defaultJointState", legged::DEFAULT_JOINT_STATE);
    ocs2::loadData::loadCppDataType(referenceFile, "targetRotationVelocity", legged::TARGET_ROTATION_VELOCITY);
    ocs2::loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", legged::TARGET_DISPLACEMENT_VELOCITY);
    ocs2::loadData::loadCppDataType(taskFile, "mpc.timeHorizon", legged::TIME_TO_TARGET);

    legged::Target target_pose_command(node, robotName);
    rclcpp::spin(node);

    return 0;
}