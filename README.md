# Cartesian Controllers

*A set of tools for commanding cartesian trajectories in ros_control*

Much of the inspiration for this package comes from [Universal_Robots_ROS_controllers_cartesian](https://github.com/UniversalRobots/Universal_Robots_ROS_controllers_cartesian).


## Installation
Cartesian Controllers depends on ruckig for online trajectory generation.
Follow the instructions at [Ruckig](https://github.com/pantor/ruckig) if ruckig is not already installed.

Clone the repository to your local workspace.
```shell script
mkdir catkin_ws/src && cd catkin_ws/src
git clone https://github.com/alexarbogast/cartesian_controllers.git
```

Build your workspace
```shell script
catkin build
```

## Trajectory Adapter

This package provides a trajectory adapter to inject an online trajectory generator into any ros_controls controller. The online trajectory generator currently uses the [Ruckig](https://github.com/pantor/ruckig) library. Generating a trajectory inside of a control loop results in better tracking than subscribing to a setpoint topic. 

The trajectory adapter creates an actionlib server that provides a trajectory execution service. When executed, the adapter stores the trajectory, and the controller can sample the trajectory online within the control loop.

Instead of providing a controller interface such as in [Universal_Robots_ROS_controllers_cartesian](https://github.com/UniversalRobots/Universal_Robots_ROS_controllers_cartesian), the trajectory adapter is meant to seperate the functionality between the trajectory managment (i.e. storage, execution, sampling) and the control. In this way, any ros_control controller_interface can easily inherit the trajectory generation functionality.

```C++
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/example_interface1.h>
#include <hardware_interface/example_interface2.h>

#include <cartesian_trajectory_adapter/cartesian_trajectory_adapter.h>

namespace controller_namespace {

class CartesianTrajectoryController : public controller_interface::MultiInterfaceController<
                                                hardware_interface::ExampleInterface1,
                                                hardware_interface::ExampleInterface1> 
                                    , public TrajectoryAdapter {
public:
    // ros_controls
    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void starting(const ros::Time&) override;
    void stopping(const ros::Time&) override;

    // trajectory adapter callback (must implement)
    void adapterStateCallback(cartesian_controllers::CartesianState& state) const;
private:
    cartesian_controllers::CartesianState setpoint_;
};

} // namespace controller_namespace
```
The controller should initialize the adapter at startup and provide a callback function that the trajectory adapter uses to find the current cartesian state of the robot.

```C++
bool CartesianTrajectoryController::init(hardware_interface::RobotHW* robot_hw,
                                         ros::NodeHandle& node_handle) {
    TrajectoryAdapter::init(node_handle, 
        boost::bind(&CartesianTrajectoryController::adapterStateCallback, this, _1));
}

void CartesianTrajectoryController::adapterStateCallback(
        cartesian_controllers::CartesianState& state) const {
    state.p = // custom impl
    state.q = // custom impl

    state.v = // custom impl
    state.w = // custom impl
}
```

Trajectories can be executed via an actionlib client. For example, in Python:
```python
from cartesian_control_msgs.msg import *

client = actionlib.SimpleActionClient('/cartesian_trajectory_controller/'
                                      'follow_cartesian_trajectory',
                                       TrajectoryExecutionAction)
```

Then, the trajectory can be sampled during the controller's update loop.

```C++
void CartesianTrajectoryController::update(const ros::Time& /*time*/, const ros::Duration& period) {
    // ================== trajectory generation ===================
    if (this->isActive() && !this->isDone()) {
        this->sample(period.toSec(), setpoint_);
    }

    // custom use of "setpoint_" in controller update
}
```
## MultiTrajectoryAdapter
The multi-trajectory adapter provides a similar interface for controllers that might need more than one trajectory generator (i.e. multi-robot controllers).  