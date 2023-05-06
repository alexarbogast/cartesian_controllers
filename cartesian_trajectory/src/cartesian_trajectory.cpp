#include <cartesian_trajectory/cartesian_trajectory.h>

namespace cartesian_controllers {

Constraints::Constraints(const cartesian_control_msgs::KinematicLimits& limits)
    : v_max(limits.v_max), a_max(limits.a_max), j_max(limits.j_max) {}

bool CartesianTrajectory::init(
    const cartesian_control_msgs::CartesianTrajectory& ros_trajectory,
    const Constraints& constraints) {
    
    trajectory_data_.clear();
    auto i = ros_trajectory.points.begin();
    for (i; std::next(i) < ros_trajectory.points.end(); ++i) {
        CartesianState state = CartesianState(*i);
        CartesianState next_state = CartesianState(*std::next(i));

        CartesianTrajectorySegment seg(state, next_state);
        trajectory_data_.push_back(seg);
    }

    constraints_ = constraints;
    return true;
}


} // namespace cartesian_controllers