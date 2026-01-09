#include "legged_estimator/legged_estimator.hpp"

namespace legged_robot{
    controller_interface::CallbackReturn legged_estimator::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn legged_estimator::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void) previous_state;
        return controller_interface::CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn legged_estimator::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void) previous_state;
        return controller_interface::CallbackReturn::SUCCESS;
    }
}