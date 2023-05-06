#pragma once

#include <actionlib/server/simple_action_server.h>
#include <cartesian_control_msgs/TrajectoryExecutionAction.h>

#include <cartesian_trajectory/cartesian_state.h>
#include <trajectory_generation/trajectory_generator.h>


namespace cartesian_trajectory_controllers {

class CartesianTrajectoryAdapter {
public:
    CartesianTrajectoryAdapter() = default;

    bool init(ros::NodeHandle& controller_nh);
    void executeCB(const cartesian_control_msgs::TrajectoryExecutionGoalConstPtr& goal);
    void preemptCB();

    void sample(cartesian_controllers::CartesianState& state); 

protected:
    using TrajectoryServer = std::unique_ptr<actionlib::SimpleActionServer
                    <cartesian_control_msgs::TrajectoryExecutionAction>>;

    TrajectoryServer action_server_;

    std::unique_ptr<cartesian_trajectory_generation::TrajectoryGenerator> gen_;
    std::atomic<bool> done_;
};

} // namespace cartesian_trajectory_controllers