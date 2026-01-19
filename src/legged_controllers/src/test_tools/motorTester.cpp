#include "legged_controllers/test_tools/motorTester.hpp"

namespace test_tools {
    controller_interface::CallbackReturn motor_tester::on_init() {
        jointNames_ = auto_declare<std::vector<std::string>>("joints", {});
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn motor_tester::on_configure(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        commandSub_ = this->get_node()->create_subscription<std_msgs::msg::String>(
            "/test_tools/motor_tester/command", 1, [this](const std_msgs::msg::String::SharedPtr msg) {
                if (commandBox_.try_set(*msg)) {
                    newCommandReceived_ = true;
                }
            });
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn motor_tester::on_activate(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        std_msgs::msg::String init_msg;
        init_msg.data = "examle_joint position 0.0";
        commandBox_.try_set(init_msg);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type motor_tester::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)time;
        (void)period;
        if (!newCommandReceived_) {
            return controller_interface::return_type::OK;
        }
        newCommandReceived_ = false;
        auto msg_opt = commandBox_.try_get();
        if (msg_opt == std::nullopt) {
            return controller_interface::return_type::OK;
        }

        std::string msg = msg_opt.value().data;
        std::vector<std::string> tokens;
        std::stringstream stream(msg);
        std::string token;
        while (std::getline(stream, token, ' ')) {
            tokens.push_back(token);
        }

        if (tokens.size() % 3 != 0) {
            RCLCPP_ERROR(get_node()->get_logger(), "Invalid command format");
            return controller_interface::return_type::OK;
        }

        for (size_t i = 0; i < tokens.size() / 3; ++i) {
            std::string joint_name = tokens[3 * i];
            std::string type = tokens[3 * i + 1];
            double cmd = std::stod(tokens[3 * i + 2]);

            // Find the corresponding command interface
            bool writeSuccess = false;
            bool found = false;
            for (size_t j = 0; j < command_interfaces_.size(); ++j) {
                if (command_interfaces_[j].get_name() == joint_name + "/" + type) {
                    writeSuccess = command_interfaces_[j].set_value<double>(cmd);
                    if (!writeSuccess) {
                        RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                                            "Failed to write to " << joint_name << "/" << type);
                    }
                    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Set " << joint_name << "/" << type << " to " << cmd);
                    found = true;
                    break;
                }
            }
            if (!found) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Joint " << joint_name << "/" << type << " not found");
            }
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::InterfaceConfiguration motor_tester::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::ALL;
        return config;
    }

    controller_interface::InterfaceConfiguration motor_tester::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::NONE;
        return config;
    }
} // namespace test_tools

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(test_tools::motor_tester, controller_interface::ControllerInterface)