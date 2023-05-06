#include <trajectory_generation/trajectory_generator.h>

namespace cartesian_trajectory_generation {

RuckigTrajectoryGenerator::RuckigTrajectoryGenerator() 
    : otg(1 / 800.0) 
{
    input.current_position = {0.0};
    input.current_velocity = {0.0};
    input.current_acceleration = {0.0};
    input.target_position = {0.0};
}

bool RuckigTrajectoryGenerator::sample(double delta_t, cartesian_controllers::CartesianState& state) {
    bool done = false;
    otg.delta_time = delta_t;
    if (otg.update(input, output) == ruckig::Result::Working) {
        auto& s = output.new_position;
        auto& v = output.new_velocity;

        state.p = current_segment_->start_state.p + s[0] * current_segment_->unit_dir;
        state.v = current_segment_->unit_dir * v[0];

        state.q = current_segment_->end_state.q;
        output.pass_to_input(input);
        return false;
    }
    
    // we are done if there are no remaining segments
    current_segment_++;
    if (current_segment_ == trajectory_.trajectory_data_.end())
        return true; 


    // reset ruckig
    input.current_position = {0.0};
    input.current_velocity = {0.0};
    input.current_acceleration = {0.0};
    input.target_position = {current_segment_->length};
    output.time = 0.0;
    return false;
}

bool RuckigTrajectoryGenerator::reset() {
    input.max_velocity = {trajectory_.constraints_.v_max};
    input.max_acceleration = {trajectory_.constraints_.a_max};
    input.max_jerk = {trajectory_.constraints_.j_max};
    current_segment_ = trajectory_.trajectory_data_.begin();

    input.current_position = {0.0};
    input.current_velocity = {0.0};
    input.current_acceleration = {0.0};
    input.target_position = {current_segment_->length};
    output.time = 0.0;
    return true;
}

} // namespace cartesian_trajectory_generation