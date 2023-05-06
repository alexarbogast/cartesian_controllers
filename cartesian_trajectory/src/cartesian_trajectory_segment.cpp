#include <cartesian_trajectory/cartesian_trajectory_segment.h>

namespace cartesian_controllers {

CartesianTrajectorySegment::CartesianTrajectorySegment(
        const CartesianState& start_state,
        const CartesianState& end_state)
    : start_state(start_state), end_state(end_state) {
        Eigen::Vector3d dir = end_state.p - start_state.p; 
        this->length = dir.norm();
        this->unit_dir = dir / this->length; 
}

} // namespace cartesian_controllers