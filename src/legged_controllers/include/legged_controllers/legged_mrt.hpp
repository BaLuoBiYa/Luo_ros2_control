#pragma once

#include "ocs2_ros_interfaces/mrt/DummyObserver.h"
#include "ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h"

namespace legged_robot {
    class legged_mrt:{
        public:
            void run(const ocs2::SystemObservation &initObservation,const ocs2::TargetTrajectories &initTargetTrajectories) override;

    };//class legged_mrt
}//namespace legged_robot