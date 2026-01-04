#include "legged_controllers/legged_wbc.hpp"
using namespace legged_robot;

controller_interface::CallbackReturn legged_wbc::on_init()
{
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

controller_interface::InterfaceConfiguration legged_wbc::state_interface_configuration() const
{

}

controller_interface::CallbackReturn legged_wbc::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    leggedInterface_ = legged_robot::LeggedInterfaceProvider::get(taskFile, urdfFile, referenceFile);
    ocs2::CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
    eeKinematicsPtr_ = std::make_shared<ocs2::PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), 
                                                                        pinocchioMapping,
                                                                        leggedInterface_->modelSettings().contactNames3DoF);

    // WBC
    wbc_ = std::make_shared<legged::WeightedWbc>(leggedInterface_->getPinocchioInterface(),
                                        leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
    wbc_->loadTasksSetting(taskFile, true);

    // Safety Checker
    safetyChecker_ = std::make_shared<legged::SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn legged_wbc::on_activate(const rclcpp_lifecycle::State &previous_state)
{

    return controller_interface::CallbackReturn::SUCCESS;
}

