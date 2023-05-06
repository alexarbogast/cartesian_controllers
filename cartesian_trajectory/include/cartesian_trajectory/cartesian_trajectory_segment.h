#pragma once

#include <cartesian_trajectory/cartesian_state.h>

namespace cartesian_controllers {

struct CartesianTrajectorySegment {
    CartesianTrajectorySegment(const CartesianState& start_state,
                               const CartesianState& end_state);
    CartesianState start_state;
    CartesianState end_state;

    Eigen::Vector3d unit_dir;
    double length;
};

} // namespace cartesian_controllers