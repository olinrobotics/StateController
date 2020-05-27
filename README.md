# Overview
The state controller is designed to act as the midbrain of a robot - facilitating easy integration of and switching between safety protocols, remote control, and fully autonomous behaviors. The controller can be configured to collect commands from an arbitrary number of nodes and send the appropriate messages through to the hindbrain. The state controller also handles activating/disactivating the robot and interacts with the softestop.

# Setup
1. Clone this repository to your catkin workspace
```
git clone https://github.com/olinrobotics/state_controller.git
```
2. Resolve dependencies
```
rosdep install -iry --from-paths src
```

# Files
Launch files:
+ `teleop.launch` - starts up the joy and teleop nodes
+ `mainstate.launch` - the primary launch file - sets up behaviors, starts up mainstate node, and runs teleop launch file.
  + Args:
    + `behaviors_file`: A .yaml file with set behaviors corresponding priorities. Defaults to behaviors listed in `config/example_behaviors`, but this should be changed for different projects


# ROS Structure
([Diagram of the ROS Nodes](https://photos.app.goo.gl/ZgS1Ykb9EHDQ4bWTA))

_graphic created using_ `rosrun rqt_graph rqt_graph`

- Topics
  - Subscribers (namespace `/state_controller`)
    - `/cmd_state`: Changes behavior being passed through the state controller (String)
    - `/cmd_activated`: Activates/disactivates robot (Bool)
    - `/cmd_behavior`: Receives control messages (TwistLabeled)
  - Publishers
    - `/cmd_twist`: Outputs control message corresponding to current behavior (Twist)
    - `/cmd_hitch`: Outputs control message corresponding to current behavior for hitch (Pose) - use z position value for height (meters off of ground, limited to -0.3 to 0.3), use y orientation for angle (degrees, limited to -45 to 45)
    - `/state`: Outputs current state controller selected state (String)

# Writing a Behavior Node
See the GitHub Wiki for tutorials and example code covering Python and C++ ([link](https://github.com/olinrobotics/state_controller/wiki))

# Known Issues
+ The Joy node assumes all axes default to 0 and will publish
0s for the two triggers until they are pulled. The trigger's "resting"
state is 1, not 0, so this becomes problematic and causes "runaway"
behavior when only one of the triggers have been pulled. To get around
this, the operator must pull both triggers to update the joy node
before the hitch will be activated
