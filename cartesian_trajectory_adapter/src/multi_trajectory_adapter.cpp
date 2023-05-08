#include <cartesian_trajectory_adapter/multi_trajectory_adapter.h>

namespace cartesian_trajectory_controllers {

bool MultiTrajectoryAdapter::init(ros::NodeHandle& controller_nh,
                                  const std::vector<std::string>& arm_ids,
                                  const StateCallback& callback) {
    
    size_t n = arm_ids.size();
    // add namespaces
    for (size_t i = 0; i < n; i++) {
        ros::NodeHandle nh_(controller_nh, arm_ids[i]);
        
        auto bound_callback = boost::bind(callback, arm_ids[i], _1);
        adapters_.emplace(std::make_pair(arm_ids[i], new CartesianTrajectoryAdapter()));
        adapters_[arm_ids[i]]->init(nh_, bound_callback);
    }
    return true;
}

} // namespace cartesian_trajectory_controllers