#include "legged_controllers/legged_wbc.hpp"
using namespace legged_robot;

controller_interface::CallbackReturn legged_wbc::on_init(){
    // Initialize parameters
    robotName = auto_declare<std::string>("robotName", "legged_robot");
    taskFile = auto_declare<std::string>("taskFile", "");
    urdfFile = auto_declare<std::string>("urdfFile", "");
    referenceFile = auto_declare<std::string>("referenceFile", "");

    joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
    if (joint_names_.size() != 12u) {
        RCLCPP_ERROR(
            get_node()->get_logger(), "Expected 12 joint names, got %zu", joint_names_.size());
        return CallbackReturn::ERROR;
    }

    visualization_enable_ = auto_declare<bool>("visualization_enable", true);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn legged_wbc::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    leggedInterface_ = std::make_shared<ocs2::legged_robot::LeggedRobotInterface>(taskFile, urdfFile, referenceFile);
    
    if (visualization_enable_){
    // Visualization
    ocs2::CentroidalModelPinocchioMapping pinocchioMapping(
        leggedInterface_->getCentroidalModelInfo());
    ocs2::PinocchioEndEffectorKinematics endEffectorKinematics(
        leggedInterface_->getPinocchioInterface(), 
        pinocchioMapping,
        leggedInterface_->modelSettings().contactNames3DoF);
    leggedRobotVisualizer_ = std::make_shared<ocs2::legged_robot::LeggedRobotVisualizer>(
        leggedInterface_->getPinocchioInterface(),
        leggedInterface_->getCentroidalModelInfo(),
        endEffectorKinematics, get_node());
    }
}
