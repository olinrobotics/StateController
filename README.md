# Overview
Kubo's main state controller is designed to take action input to the tractor from the joystick, safety nodes, and behavior nodes, and send the appropriate messages through to the hindbrain. The state controller also handles activating/disactivating the tractor and interacts with the softestop.

# ROS Structure
([Diagram of the ROS Nodes](https://photos.app.goo.gl/NhwqovNdoa92GxE2A))

<!--<img src="https://photos.google.com/share/AF1QipM8WilQQL3N2mR3JRYb8OCD1ecl4_slz66VOQBlAzbU90pZNpKQ_5MJiphqgePrqA/photo/AF1QipNinHB2Ymm0YtTOc-QuCwP19LvkYGyDeG108VmU?key=Q3hJeDk1c0FnZVVpUHAtS2pLbmV4TG1USnV1RGZn" width=1000/>  -->

_graphic created using_ `rosrun rqt_graph rqt_graph`

- Topics
  - Subscribers
    - `/cmd_state`: Changes behavior being passed through the state controller (String)
    - `/cmd_activated`: Activates/disactivates tractor (Bool)
    - `/cmd_behavior`: Receives control messages (TwistLabeled)
  - Publishers
    - `/cmd_twist`: Outputs control message corresponding to current behavior (Twist)
    - `/state`: Outputs current tractor state (String)

# Writing a Behavior Node
See Wiki Page ([link](https://github.com/olinrobotics/state_controller/wiki/Tutorials-WritingBehaviorNode(c--)))
