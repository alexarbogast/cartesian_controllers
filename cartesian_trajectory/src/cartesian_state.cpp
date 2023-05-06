#include <cartesian_trajectory/cartesian_state.h>

namespace cartesian_controllers {

CartesianState::CartesianState() {
    p = Eigen::Vector3d::Zero();
    q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

    v = Eigen::Vector3d::Zero();
    w = Eigen::Vector3d::Zero();
}

CartesianState::CartesianState(
    const cartesian_control_msgs::CartesianTrajectoryPoint point) {
    
    p = Eigen::Vector3d(point.pose.position.x, 
                        point.pose.position.y,
                        point.pose.position.z);
    q = Eigen::Quaterniond(point.pose.orientation.w,
                           point.pose.orientation.x,
                           point.pose.orientation.y,
                           point.pose.orientation.z);
    v = Eigen::Vector3d(point.twist.linear.x, 
                        point.twist.linear.y,
                        point.twist.linear.z);
    w = Eigen::Vector3d(point.twist.angular.x, 
                        point.twist.angular.y,
                        point.twist.angular.z);
}

std::ostream& operator<<(std::ostream& out, const CartesianState& state)
{
  out << "p:\n" << state.p << '\n';
  out << "q:\n" << state.q.coeffs() << '\n';
  out << "v:\n" << state.v << '\n';
  out << "w:\n" << state.w << '\n';
  return out;
}

} // namespace cartesian_controllers