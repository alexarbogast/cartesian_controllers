#pragma once

#include <mutex>
#include <cartesian_trajectory/cartesian_trajectory.h>
#include <cartesian_control_msgs/CartesianTrajectory.h>

// abstract this
#include <ruckig/ruckig.hpp>

namespace cartesian_trajectory_generation {

class TrajectoryGenerator {
public:

    virtual bool sample(double delta_t, cartesian_controllers::CartesianState& state) = 0;
    virtual bool reset() = 0;

    cartesian_controllers::CartesianTrajectory trajectory_;

    using SegmentIterator = std::vector<cartesian_controllers::CartesianTrajectorySegment>::iterator;
    SegmentIterator current_segment_;
};


class RuckigTrajectoryGenerator : public TrajectoryGenerator {
public:
    RuckigTrajectoryGenerator();

    virtual bool sample(double delta_t, cartesian_controllers::CartesianState& state) override;
    virtual bool reset() override;

protected:
    const static size_t DOFs {1};
    ruckig::Ruckig<DOFs> otg;
    ruckig::InputParameter<DOFs> input;
    ruckig::OutputParameter<DOFs> output;
};

} // namespace cartesian_trajectory_generation