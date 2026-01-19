#pragma once
#include <controller_interface/semantic_components/imu_sensor.hpp>
#include <eigen3/Eigen/Dense>

namespace legged {
    class IMUhandler : public semantic_components::IMUSensor {
      public:
        IMUhandler(const std::string &name) : semantic_components::IMUSensor(name) {}

        std::array<double, 9> get_orientation_covariance() {
            return std::array<double, 9>();
        }
        std::array<double, 9> get_angular_velocity_covariance() const;
        std::array<double, 9> get_linear_acceleration_covariance() const;

      private:
    };
} // namespace legged