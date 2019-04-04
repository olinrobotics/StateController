#ifndef MAIN_STATE_H
#define MAIN_STATE_H

#include <ros/ros.h>
#include <midbrain_sc/TwistLabeled.h>
#include <midbrain_sc/ArrayLabeled.h>
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
  void behaviorCB(const midbrain_sc::TwistLabeled& msg);
  void behavior2CB(const midbrain_sc::ArrayLabeled& msg);
  void setState(std_msgs::String state);
  void addBehavior(std::pair<std::string, std::string> pair);
  void updateBehaviors();
  int getBehaviorPriority(std_msgs::String label, int* priority);
};

#endif //MAIN_STATE_H
