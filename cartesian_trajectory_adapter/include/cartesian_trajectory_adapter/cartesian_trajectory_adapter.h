#pragma once

#include <actionlib/server/simple_action_server.h>
#include <cartesian_control_msgs/TrajectoryExecutionAction.h>

#include <cartesian_trajectory/cartesian_state.h>
#include <trajectory_generation/trajectory_generator.h>


#define LOCK_ADAPTER_DATA() std::lock_guard<std::mutex> lock_setpoint(adapter_lock_)

namespace cartesian_trajectory_controllers {

class CartesianTrajectoryAdapter {
public:
    CartesianTrajectoryAdapter() = default;

    using StateCallback = boost::function<void(cartesian_controllers::CartesianState&)>;
    bool init(ros::NodeHandle& controller_nh, 
              const StateCallback& callback);
    void executeCB(const cartesian_control_msgs::TrajectoryExecutionGoalConstPtr& goal);
    void preemptCB();

    using TrajectoryServer = std::unique_ptr<actionlib::SimpleActionServer
                    <cartesian_control_msgs::TrajectoryExecutionAction>>;

    inline bool isDone() const { return done_.load(); }
    inline bool isActive() const { return action_server_->isActive(); }

    bool sample(double delta_t, cartesian_controllers::CartesianState& state);

    std::mutex adapter_lock_;
protected:
    std::unique_ptr<cartesian_trajectory_generation::TrajectoryGenerator> gen_;

    StateCallback state_callback_;
    TrajectoryServer action_server_;
    std::atomic<bool> done_;
};

} // namespace cartesian_trajectory_controllers