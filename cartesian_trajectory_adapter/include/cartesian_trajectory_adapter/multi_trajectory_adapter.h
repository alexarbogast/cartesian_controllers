#pragma once

#include <cartesian_trajectory_adapter/cartesian_trajectory_adapter.h>
#include <unordered_map>

#define LOCK_MULTI_ADAPTER_DATA(arm_id) std::lock_guard<std::mutex> lock_adapter(adapters_.at(arm_id)->adapter_lock_);

namespace cartesian_trajectory_controllers {

class MultiTrajectoryAdapter {
public:
    MultiTrajectoryAdapter() = default;
    
    using StateCallback = boost::function<void (const std::string&, 
                          cartesian_controllers::CartesianState&)>;
    
    bool init(ros::NodeHandle& controller_nh,
              const std::vector<std::string>& arm_ids,
              const StateCallback& callback_);

protected:
    using AdapterPtr = std::unique_ptr<CartesianTrajectoryAdapter>;
    std::unordered_map<std::string, AdapterPtr> adapters_;

    StateCallback state_callback_;
};

} // namespace cartesian_trajectory_controllers