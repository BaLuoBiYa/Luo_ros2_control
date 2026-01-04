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

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration legged_wbc::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(24 + 24 + 24 +1);

    const std::vector<std::string> optimizedStateNames = {
            "vcom_x","vcom_y","vcom_z",
            // linear momentum
            "L_x","L_y","L_z",
            // angular momentum
            "p_base_x","p_base_y","p_base_z",
            // base postion
            "theta_base_z","theta_base_y","theta_base_x",
            // base orientation
            "LF_HAA","LF_HFE","LF_KFE",
            // left front leg positions
            "LH_HAA","LH_HFE","LH_KFE",
            // left hind leg positions
            "RF_HAA","RF_HFE","RF_KFE",
            // right front leg positions
            "RH_HAA","RH_HFE","RH_KFE"
            // right hind leg positions
        };
    for (const auto &name : optimizedStateNames){
        config.names.push_back("optimizedState/" + name);
    }

    const std::vector<std::string> optimizedInputNames = {
            "left_front_force_x","left_front_force_y","left_front_force_z",
            "right_front_force_x","right_front_force_y","right_front_force_z",
            "left_hind_force_x","left_hind_force_y","left_hind_force_z",
            "right_hind_force_x","right_hind_force_y","right_hind_force_z",
            // contact forces
            "LF_x","LF_y","LF_z",
            "LH_x","LH_y","LH_z",
            "RF_x","RF_y","RF_z",
            "RH_x","RH_y","RH_z"
            // feet contact forces
        };
    for (const auto &name : optimizedInputNames){
        config.names.push_back("optimizedInput/" + name);
    }

    const std::vector<std::string> rbdStateNames = {
            "base_roll","base_pitch","base_yaw",
            // base orientation
            "base_pos_x","base_pos_y","base_pos_z",
            // base position
            "LF_HAA_pos","LF_HFE_pos","LF_KFE_pos",
            // left front leg joint positions
            "LH_HAA_pos","LH_HFE_pos","LH_KFE_pos",
            // left hind leg joint positions
            "RF_HAA_pos","RF_HFE_pos","RF_KFE_pos",
            // right front leg joint positions
            "RH_HAA_pos","RH_HFE_pos","RH_KFE_pos",
            // right hind leg joint positions

            "base_angular_vel_x","base_angular_vel_y","base_angular_vel_z",
            // base angular velocity
            "base_linear_vel_x","base_linear_vel_y","base_linear_vel_z",
            // base linear velocity
            "LF_HAA_vel","LF_HFE_vel","LF_KFE_vel",
            // left front leg joint velocities
            "LH_HAA_vel","LH_HFE_vel","LH_KFE_vel",
            // left hind leg joint velocities
            "RF_HAA_vel","RF_HFE_vel","RF_KFE_vel",
            // right front leg joint velocities
            "RH_HAA_vel","RH_HFE_vel","RH_KFE_vel"
            // right hind leg joint velocities
        };
    for (const auto &name : rbdStateNames){
        config.names.push_back("rbdState/" + name);
    }

    config.names.push_back("desiredMode/contact_state");
    return config;
}

controller_interface::InterfaceConfiguration legged_wbc::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(12 + 12 + 12);
    for (const auto &joint_name : joint_names_){
        config.names.push_back(joint_name + "/position");
    }
    //申请关节位置命令接口
    for (const auto &joint_name : joint_names_){
        config.names.push_back(joint_name + "/velocity");
    }
    //申请关节速度命令接口
    for (const auto &joint_name : joint_names_){
        config.names.push_back(joint_name + "/effort");
    }
    //申请关节力命令接口
    return config;
}

controller_interface::CallbackReturn legged_wbc::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void) previous_state;

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

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn legged_wbc::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void) previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type legged_wbc::update_reference_from_subscribers(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void) time;
    (void) period;
    return controller_interface::return_type::OK;
}

controller_interface::return_type legged_wbc::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void) time;
    (void) period;
    for (size_t i = 0; i < 24; i++){
        optimizedState_(i) = state_interfaces_[i].get_optional().value();
        optimizedInput_(i) = state_interfaces_[i + 24].get_optional().value();
        rbdState_(i) = state_interfaces_[i + 48].get_optional().value();
    }
    plannedMode_ = static_cast<size_t>(state_interfaces_[72].get_optional().value());

    ocs2::vector_t command = wbc_->update(optimizedState_, optimizedInput_, rbdState_, plannedMode_, period.seconds());
    ocs2::vector_t torque = command.tail(12);
    ocs2::vector_t posDes = ocs2::centroidal_model::getJointAngles(optimizedState_, leggedInterface_->getCentroidalModelInfo());
    ocs2::vector_t velDes = ocs2::centroidal_model::getJointVelocities(optimizedInput_, leggedInterface_->getCentroidalModelInfo());

    bool commandValid = true;
    for (size_t i = 0; i < 12; i++){
        commandValid = command_interfaces_[i].set_value(posDes(i));
        commandValid = command_interfaces_[i + 12].set_value(velDes(i));
        commandValid = command_interfaces_[i + 24].set_value(torque(i));
    }

    if (commandValid)
    {
        return controller_interface::return_type::OK;
    }
    return controller_interface::return_type::ERROR;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(legged_robot::legged_wbc, controller_interface::ChainableControllerInterface)

