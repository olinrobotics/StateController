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

# 1. Writing a Behavior Node
The following instructions detail how to create a behavior that interfaces with the tractor's state controller. First, let's create a basic behavior node (example code adapted from ROS c++ PublisherSubscriber tutorial ([link](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)):

## 1.1 Code
Create a MyBehavior directory in the gravl/src directory:
```
../catkin_ws/src/gravl/src$ mkdir MyBehavior
```
Create the src/BehaviorEx.h file within your new directory and paste the following inside it:
```
/*
 * @file BehaviorEx.cpp
 * @brief Example behavior, publishes command received on /ex_topic
 *
 * Subscribes to /ex_topic, publishes received msgs to state controller via
 * /cmd_behavior
 * @author Connor Novak
 * @date 2018-11-13
 */

#ifndef BEHAVIOR_EX_H
#define BEHAVIOR_EX_H

#include <ros/ros.h>
#include <gravl/TwistLabeled.h>

class BehaviorEx {
  public:
    explicit BehaviorEx();
  private:
    ros::NodeHandle n;
    ros::Subscriber twist_subscriber;
    ros::Publisher twist_publisher;

    const char* name;
    gravl::TwistLabeled current_message;

    void twistCB(const geometry_msgs::Twist& msg);
};

#endif

```
Create the src/BehaviorEx.cpp file within your new directory and paste the following inside it:  
```
/*
 * @file BehaviorEx.cpp
 * @author Connor Novak
 * @date 2018-12-12
 *
 * Subscribes to /cmd_behavior, publishes msgs to /cmd_vel based on tractor's
 * current state.
 */

 #include "BehaviorEx.h"          // header
 #include <geometry_msgs/Twist.h> // node lisens to twist messages
 #include <gravl/TwistLabeled.h>  // node publishes twistlabeled messages

 BehaviorEx::BehaviorEx() {
    twist_subscriber = n.subscribe("/ex_topic", 1, &BehaviorEx::BehaviorEx::twistCB, this);
    twist_publisher = n.advertise<gravl::TwistLabeled>("/state_controller/cmd_behavior", 1);
    current_message.label = "example";
 }

void BehaviorEx::twistCB(const geometry_msgs::Twist& msg) {
  /*
   * @brief callback function called for every command message received
   *
   * @param[in] twist message from subscriber
   * param[out] updates current_message class attribute
   */

   current_message.twist = msg;
   twist_publisher.publish(current_message);

}

int main (int argc, char** argv) {
  ros::init(argc, argv, "BehaviorEx");  //Initialize the ROS node
  BehaviorEx behavior;                  // Initialize a behavior class instance
  ros::spin();                          // Continuously update the ROS loop
}
```

## 1.2 The Code Explained
Let's start by breaking down the header file.
```
#ifndef BEHAVIOR_EX_H
#define BEHAVIOR_EX_H

...

#endif
```
This is a header guard, preventing the header for BehaviorEx from being defined multiple times if it is included by multiple files. It ensures that the file is only defined if the system variable `BEHAVIOR_EX_H` is defined.
```
#include <ros/ros.h>
#include <gravl/TwistLabeled.h>
```
ros/ros.h is a convenience include that includes all the headers necessary to use the most common public pieces of the ROS system. gravl/TwistLabeled.h is a custom ROSmessage defined for the gravl repository that represents a Twist message with a string label representing which behavior is publishing the message. This string should match the string in `/config/tractor_behaviors.yaml`.
```
public:
    explicit BehaviorEx();
```
The only publicly available function for class BehaviorEx is the constructor, required to instantiate an instance of the class.
```
    ros::NodeHandle n;
    ros::Subscriber twist_subscriber;
    ros::Publisher twist_publisher;

    gravl::TwistLabeled current_message;
```
BehaviorEx has a number of private attributes available for its methods to access. It has its NodeHandle, a publisher, and a subscriber all defined here for initialization in the class constructor. The current_message attribute holds the currently executing command.

Now, let's dig into the .cpp file, the place where the bulk of the code is written.
```
 #include "BehaviorEx.h"          // header
 #include <geometry_msgs/Twist.h> // node lisens to twist messages
 #include <gravl/TwistLabeled.h>  // node publishes twistlabeled messages
```
These includes define all of the packages that we would need to run the .cpp file if we didn't define the header. Twist and TwistLabeled messages are used to send and receive data, and the header file needs to be included.
```
 BehaviorEx::BehaviorEx() {
    twist_subscriber = n.subscribe("/ex_topic", 1, &BehaviorEx::BehaviorEx::twistCB, this);
    twist_publisher = n.advertise<gravl::TwistLabeled>("/state_controller/cmd_behavior", 1);
    current_message.label = "example";
 }
```
The constructor for the class instantiates a subscriber that listens to the topic `/ex_topic` for Twist messages, and calls the method `twistCB` every time it receives a message. the constructor also initiates a publisher that publishes to the command behavior topic for the state controller. Finally, the constructor initializes the string name for the current command such that all send messages have the "example" tag.
```
void BehaviorEx::twistCB(const geometry_msgs::Twist& msg) {

   current_message.twist = msg;
   twist_publisher.publish(current_message);

}
```
The twistCB is a callback function for the twist_subscriber. A callback function in ROS runs every time a subscriber receives a message. They are usually used for processing and storing data, and frequently need to run much faster than the main code. This callback function merely stores the incoming Twist message in the outgoing TwistLabeled message, the publishes the resulting message.
```
int main (int argc, char** argv) {
  ros::init(argc, argv, "BehaviorEx");  // Initialize the ROS node
  BehaviorEx behavior;                  // Initialize a behavior class instance
  ros::spin();                          // Continuously update the ROS loop
}
```
The main function of the program is run when the .cpp file is run individually, rather than being imported into a separate script. In this case, the script initializes a ros node called BehaviorEx, initializes and instance of the Behavior node, and then runs ros spin to allow the callback functions to be continually updated.

# 2. Building your nodes
The tractor package already has a CMakeLists.txt built, so all that needs to be done is to add BehaviorEx to the file such that it is compiled with the rest of the repository. Add the following lines:
```
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

...

add_executable(BehaviorEx src/BehaviorEx/BehaviorEx.cpp)
```
This line points to BehaviorEx.cpp as an executable file that catkin needs to ensure compiles properly.
```
## Specify libraries to link a library or executable target against

...

target_link_libraries(BehaviorEx ${catkin_LIBRARIES})
```
This line allows the .cpp file to access the ROS libraries such that it has all of the information necessary to compile.

Now run catkin_make:
```
# In your catkin workspace
$ cd ~/catkin_ws
$ catkin_make
```

3. Registering Behavior
The last step in writing a new behavior for the tractor is to register your behavior with the main state controller such that the state controller doesn't discard messages received from a behavior it does not recognize. Add your behavior to `gravl/config/tractor_behaviors.yaml`, following the structure outlined in the comments.
