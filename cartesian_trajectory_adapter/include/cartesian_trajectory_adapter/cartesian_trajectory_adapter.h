#pragma once

#include <actionlib/server/simple_action_server.h>
#include <cartesian_control_msgs/TrajectoryExecutionAction.h>

#include <cartesian_trajectory/cartesian_state.h>
#include <trajectory_generation/trajectory_generator.h>


namespace cartesian_trajectory_controllers {
using StateHandle = const cartesian_controllers::CartesianState*;

class CartesianTrajectoryAdapter {
public:
    CartesianTrajectoryAdapter() = default;

    bool init(ros::NodeHandle& controller_nh, const StateHandle state_handle);
    void executeCB(const cartesian_control_msgs::TrajectoryExecutionGoalConstPtr& goal);
    void preemptCB();

    using TrajectoryServer = std::unique_ptr<actionlib::SimpleActionServer
                    <cartesian_control_msgs::TrajectoryExecutionAction>>;

    inline bool isDone() const { return done_.load(); }
    inline bool isActive() const { return action_server_->isActive(); }

    bool sample(double delta_t, cartesian_controllers::CartesianState& state);

protected:
    std::unique_ptr<cartesian_trajectory_generation::TrajectoryGenerator> gen_;

    TrajectoryServer action_server_;
    StateHandle state_handle_;
    std::atomic<bool> done_;
};

} // namespace cartesian_trajectory_controllers