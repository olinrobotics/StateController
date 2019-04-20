# Overview
Kubo's main state controller is designed to take action input to the tractor from the joystick, safety nodes, and behavior nodes, and send the appropriate messages through to the hindbrain. The state controller also handles activating/disactivating the tractor and interacts with the softestop.

# ROS Structure
([Diagram of the ROS Nodes](https://photos.app.goo.gl/ZgS1Ykb9EHDQ4bWTA))

_graphic created using_ `rosrun rqt_graph rqt_graph`

- Topics
  - Subscribers (namespace `/state_controller`)
    - `/cmd_state`: Changes behavior being passed through the state controller (String)
    - `/cmd_activated`: Activates/disactivates tractor (Bool)
    - `/cmd_behavior`: Receives control messages (TwistLabeled)
  - Publishers
    - `/cmd_twist`: Outputs control message corresponding to current behavior (Twist)
    - `/cmd_array`: Outputs control message corresponding to current behavior (Array)
    - `/state`: Outputs current tractor state (String)

# Writing a Behavior Node
See Wiki Page ([link](https://github.com/olinrobotics/state_controller/wiki/Tutorial:WritingBehaviorNode(Cpp)))
