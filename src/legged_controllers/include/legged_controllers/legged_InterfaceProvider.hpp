#pragma once
#include <memory>
#include <mutex>
#include <ocs2_legged_robot/LeggedRobotInterface.h>

namespace legged_robot {
class LeggedInterfaceProvider {
public:
  static std::shared_ptr<ocs2::legged_robot::LeggedRobotInterface> get(
      const std::string& task, const std::string& urdf, const std::string& ref) 
    {
      static std::mutex mtx;
      static std::weak_ptr<ocs2::legged_robot::LeggedRobotInterface> weak;
      std::lock_guard<std::mutex> lock(mtx);

      if (auto sp = weak.lock()) {
        return sp;
      }
      
      auto created = std::make_shared<ocs2::legged_robot::LeggedRobotInterface>(task, urdf, ref);
      weak = created;
      return created;
    }
};
}  // namespace legged_robot