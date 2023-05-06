#pragma once

#include <cartesian_trajectory/cartesian_trajectory_segment.h>
#include <cartesian_control_msgs/CartesianTrajectory.h>
#include <cartesian_control_msgs/KinematicLimits.h>

namespace cartesian_controllers {

struct Constraints {
    Constraints() = default;
    Constraints(const cartesian_control_msgs::KinematicLimits& limits);

    double v_max;
    double a_max;
    double j_max;
};


struct CartesianTrajectory {
    CartesianTrajectory() = default;

    bool init(const cartesian_control_msgs::CartesianTrajectory& ros_trajectory,
              const Constraints& constraints);

    Constraints constraints_;
    std::vector<CartesianTrajectorySegment> trajectory_data_;
};

} // namespace cartesian_controllers