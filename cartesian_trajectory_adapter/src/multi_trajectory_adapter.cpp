#include <cartesian_trajectory_adapter/multi_trajectory_adapter.h>

namespace cartesian_trajectory_controllers {

bool MultiTrajectoryAdapter::init(ros::NodeHandle& controller_nh,
                                  const std::vector<std::string>& arm_ids,
                                  const std::vector<StateHandle>& state_handles) {
    size_t n = arm_ids.size();
    if (n != state_handles.size()) {
        ROS_ERROR("The size of arm_ids does not match the size of state_handles");
        return false;
    }
    
    // add namespaces
    for (size_t i = 0; i < n; i++) {
        ros::NodeHandle nh_(controller_nh, arm_ids[i]);
        adapters_.emplace(std::make_pair(arm_ids[i], new CartesianTrajectoryAdapter()));
        adapters_[arm_ids[i]]->init(nh_, state_handles[i]);
    }
    return true;
}

} // namespace cartesian_trajectory_controllers