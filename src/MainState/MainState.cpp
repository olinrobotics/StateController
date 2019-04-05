/*
 * @file MainState.cpp
 * @author Connor Novak
 * @date 2018-11-13
 *
 * Subscribes to /cmd_behavior, publishes msgs to /cmd_vel based on tractor's
 * current state.
 */

#include "MainState.h"
#include <ros/console.h>  // Used for logging
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <vector>
#include <state_controller/Array.h>
#include <typeinfo>


MainState::MainState()
 : rate(ros::Rate(2))
 , state_sub(n.subscribe("/state_controller/cmd_state", 1, &MainState::MainState::stateCB, this))
 , activate_sub(n.subscribe("/state_controller/cmd_activate", 1, &MainState::MainState::activateCB, this))
 , behavior_sub(n.subscribe("/state_controller/cmd_behavior", 10, &MainState::MainState::behaviorCB, this))
 , behavior2_sub(n.subscribe("/state_controller/cmd_behavior2", 10, &MainState::MainState::behavior2CB, this))
 , state_pub(n.advertise<std_msgs::String>("state", 1))
 , command_pub(n.advertise<geometry_msgs::Twist>("cmd_twist", 1))
 , command2_pub(n.advertise<state_controller::Array>("cmd_array",1))
 , curr_state()
 , is_activated(false)
 , behavior_map(getBehaviorMap(n)) {
   curr_state.data = "safety";
}

void MainState::stateCB(const std_msgs::String& msg) {
  // Callback for joy_state, updates and publishes curr_state
    if (msg.data != curr_state.data) setState(msg);
}

void MainState::activateCB(const std_msgs::Bool& msg) {
  // Callback for joy_active, updates activated state
  is_activated = msg.data;
  if (is_activated) ROS_INFO("Activating Tractor");
  else {ROS_INFO("Disactivating Tractor");}
}

void MainState::behaviorCB(const state_controller::TwistLabeled& msg) {
  /*
*/

  // Get priority of behavior arg
  auto msg_behavior = behavior_map[msg.label.data];
  auto priority = msg_behavior->getPriority();

  // Estop overwrites Teleop overwrites Bn
  if (priority == 0) setState(msg.label);
  else if (priority == 1 && behavior_map[curr_state.data]->getPriority() != 0) setState(msg.label);

  // Update behavior, publish message
  msg_behavior->setMessage(msg);
  if (msg.label.data == curr_state.data) {
    command_pub.publish(msg.twist);
  }
}

void MainState::behavior2CB(const state_controller::ArrayLabeled& msg) {
/*
*/
  // Get priority of given behavior
  // Get priority of behavior arg
  auto msg_behavior = behavior_map[msg.label.data];
  auto priority = msg_behavior->getPriority();

  // Estop overwrites Teleop overwrites Bn
  if (priority == 0) setState(msg.label);
  else if (priority == 1 && behavior_map[curr_state.data]->getPriority() != 0) setState(msg.label);

  // Update behavior, publish message
  msg_behavior->setMessage(msg);
  state_controller::Array messageToBePublished;
  if (msg.label.data == curr_state.data) {
    messageToBePublished.data = msg.data;
    command2_pub.publish(messageToBePublished);
  }
}

void MainState::setState(std_msgs::String state) {
  /*! \brief Updates state with new state.
  *
  * Note: this takes in a ROS string
  * setState updates the current state with a given state, publishes an info
  * message, and publishes the new state to the topic /curr_state
  */

  if (curr_state.data != state.data) {
    curr_state = state;
    ROS_INFO("Activating State :%s", state.data.c_str());
    state_pub.publish(curr_state);
  }
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "MainState");
  MainState mainstate;
  ros::spin();
}
