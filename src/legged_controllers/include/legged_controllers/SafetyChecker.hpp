//
// Created by qiayuan on 2022/7/26.
//

#pragma once

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_mpc/SystemObservation.h>

namespace legged
{
	class SafetyChecker
	{
	public:
		explicit SafetyChecker(const ocs2::CentroidalModelInfo &info) : info_(info) {}

		bool check(const ocs2::SystemObservation &observation, const ocs2::vector_t & /*optimized_state*/, const ocs2::vector_t & /*optimized_input*/)
		{
			return checkOrientation(observation);
		}

	protected:
		bool checkOrientation(const ocs2::SystemObservation &observation)
		{
			ocs2::vector_t pose = ocs2::centroidal_model::getBasePose(observation.state, info_);
			if (pose(5) > M_PI_2 || pose(5) < -M_PI_2)
			{
				std::cerr << "[SafetyChecker] Orientation safety check failed!" << std::endl;
				return false;
			}
			return true;
		}

		const ocs2::CentroidalModelInfo &info_;
	};

} // namespace legged
