#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <iomanip>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> //#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> doesn't work on Ubuntu20.04
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sensor_msgs/msg/imu.hpp>

#include "fusion_estimator/msg/fusion_estimator_test.hpp"
#include "GO2FusionEstimator/SensorBase.h"
#include "GO2FusionEstimator/Sensor_Legs.h"
#include "GO2FusionEstimator/Sensor_IMU.h"