#include "legged_controllers/test_tools/contactTester.hpp"

namespace test_tools {
    controller_interface::CallbackReturn contact_tester::on_init() {
        contactNames_ = auto_declare<std::vector<std::string>>("contactNames", {});
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn contact_tester::on_configure(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Pub =
            this->get_node()->create_publisher<std_msgs::msg::String>("/test_tools/contact_tester/state",
                                                                      rclcpp::QoS(1));

        statePub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(Pub);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn contact_tester::on_activate(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        std_msgs::msg::String msg;
        msg.data = "Contact tester activated.";
        if (statePub_->trylock()) {
            statePub_->msg_ = msg;
            statePub_->unlockAndPublish();
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type contact_tester::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)time;
        (void)period;
        std_msgs::msg::String msg;
        for (size_t i = 0; i < contactNames_.size(); ++i) {
            auto contact = state_interfaces_[i].get_optional<bool>();
            if (contact == std::nullopt) {
                RCLCPP_ERROR_STREAM(this->get_node()->get_logger(),
                                    "Contact state interface " << contactNames_[i] << " is not available.");
                continue;
            }
            if (contact.value() == true) {
                msg.data += contactNames_[i] + ":true\n";
            } else {
                msg.data += contactNames_[i] + ":false\n";
            }
        }

        if (statePub_->trylock()) {
            statePub_->msg_ = msg;
            statePub_->unlockAndPublish();
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::InterfaceConfiguration contact_tester::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::NONE;
        return config;
    }

    controller_interface::InterfaceConfiguration contact_tester::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (const auto &contactName : contactNames_) {
            config.names.push_back("contact/" + contactName);
        }
        return config;
    }
} // namespace test_tools

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(test_tools::contact_tester, controller_interface::ControllerInterface)