#ifndef MAIN_STATE_H
#define MAIN_STATE_H

#include <ros/ros.h>
#include <state_controller/TwistLabeled.h>
#include <state_controller/ArrayLabeled.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include "Behavior.h"
#include <map>

class MainState{
public:
  explicit MainState();
private:
  ros::NodeHandle n;
  ros::Subscriber state_sub;
  ros::Subscriber activate_sub;
  ros::Subscriber behavior_sub;
  ros::Subscriber behavior2_sub;
  ros::Publisher state_pub;
  ros::Publisher command_pub;
  ros::Publisher command2_pub;
  ros::Rate rate;
  std_msgs::String curr_state;

  std::map<std::string, Behavior*> behavior_map;
  bool is_activated;

  void stateCB(const std_msgs::String& msg);
  void activateCB(const std_msgs::Bool& msg);
  void behaviorCB(const state_controller::TwistLabeled& msg);
  void behavior2CB(const state_controller::ArrayLabeled& msg);
  void setState(std_msgs::String state);
  void addBehavior(std::pair<std::string, std::string> pair);
  void updateBehaviors();
  int getBehaviorPriority(std_msgs::String label, int* priority);
};

#endif //MAIN_STATE_H
