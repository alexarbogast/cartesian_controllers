#include <cartesian_trajectory_adapter/cartesian_trajectory_adapter.h>


namespace cartesian_trajectory_controllers {

bool CartesianTrajectoryAdapter::init(ros::NodeHandle& controller_nh,
                                      const StateHandle state_handle) {
    done_ = true;
    state_handle_ = state_handle;

    gen_.reset(new cartesian_trajectory_generation::RuckigTrajectoryGenerator());
    action_server_.reset(new actionlib::SimpleActionServer<
                             cartesian_control_msgs::TrajectoryExecutionAction>(
                                controller_nh, "follow_cartesian_trajectory",
                                boost::bind(&CartesianTrajectoryAdapter::executeCB, this, _1), 
                                false));

    action_server_->registerPreemptCallback(boost::bind(&CartesianTrajectoryAdapter::preemptCB, this));
    action_server_->start();
    return true;
}

bool CartesianTrajectoryAdapter::sample(double delta_t, 
        cartesian_controllers::CartesianState& state) {
    std::lock_guard<std::mutex> lock_trajectory(gen_->traj_lock_);
    this->done_ = gen_->sample(delta_t, state);
    return done_;
}

void CartesianTrajectoryAdapter::executeCB(
        const cartesian_control_msgs::TrajectoryExecutionGoalConstPtr& goal) {
    cartesian_control_msgs::TrajectoryExecutionResult result;
    {
        std::lock_guard<std::mutex> lock_trajectory(gen_->traj_lock_);
        cartesian_control_msgs::CartesianTrajectory traj = goal->trajectory;

        auto current_state = this->state_handle_->toMsg();
        traj.points.insert(traj.points.begin(), this->state_handle_->toMsg());

        if (!gen_->trajectory_.init(traj, goal->limits)) {
            ROS_ERROR("Trajectory Adapter: Invalid trajectory");
            result.error_string = "invalid trajectory";
            action_server_->setAborted(result);
            return;
        }
        gen_->reset();
    }

    done_ = false;
    while (!done_.load()) {
        ros::Duration(0.1).sleep();
    }
    action_server_->setSucceeded(result);
}

void CartesianTrajectoryAdapter::preemptCB() {
    cartesian_control_msgs::TrajectoryExecutionResult result;
    result.error_string = "preempted";
    action_server_->setPreempted(result);
}

} // namespace cartesian_trajectory_controllers