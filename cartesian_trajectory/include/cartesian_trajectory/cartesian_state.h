#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cartesian_control_msgs/CartesianTrajectoryPoint.h>

namespace cartesian_controllers {

struct CartesianState {
    CartesianState();
    CartesianState(const cartesian_control_msgs::CartesianTrajectoryPoint point);

    cartesian_control_msgs::CartesianTrajectoryPoint toMsg() const;

    Eigen::Vector3d p;
    Eigen::Quaterniond q;

    Eigen::Vector3d v;
    Eigen::Vector3d w;

    friend std::ostream& operator<<(std::ostream& os, const CartesianState& state);
};

} // namespace cartesian_controllers