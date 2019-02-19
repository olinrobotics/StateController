#ifndef MAIN_STATE_H
#define MAIN_STATE_H

#include <ros/ros.h>
#include <midbrain_sc/TwistLabeled.h>
#include <midbrain_sc/ArrayLabeled.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include "Behavior.h"
#include <vector>

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
  std_msgs::UInt8 curr_state;

  std::vector<Behavior> behavior_vector;
  bool is_activated;

  void stateCB(const std_msgs::UInt8& msg);
  void activateCB(const std_msgs::Bool& msg);
  void behaviorCB(const midbrain_sc::TwistLabeled& msg);
  void behavior2CB(const midbrain_sc::ArrayLabeled& msg);
  void setState(std_msgs::UInt8 state);
  void setState(int state);
  void addBehavior(std::pair<std::string, std::string> pair);
  void updateBehaviors();
  int getBehavior(int label, int* index);
};

#endif //MAIN_STATE_H
