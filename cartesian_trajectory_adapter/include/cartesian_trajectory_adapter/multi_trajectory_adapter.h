#pragma once

#include <cartesian_trajectory_adapter/cartesian_trajectory_adapter.h>
#include <unordered_map>

namespace cartesian_trajectory_controllers {

class MultiTrajectoryAdapter {
public:
    MultiTrajectoryAdapter() = default;
    
    bool init(ros::NodeHandle& controller_nh,
              const std::vector<std::string>& arm_ids,
              const std::vector<StateHandle>& state_handle);

protected:
    using AdapterPtr = std::unique_ptr<CartesianTrajectoryAdapter>;
    std::unordered_map<std::string, AdapterPtr> adapters_;
};

} // namespace cartesian_trajectory_controllers