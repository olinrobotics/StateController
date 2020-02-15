/*
 * @file Teleop.h
 * @brief function prototypes for teleop node
 *
 * @author Carl Moser
 * #maintainer Olin GRAVL
 * @email olingravl@gmail.com
 */

#ifndef TELEOP_H
#define TELEOP_H

#include <string.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <state_controller/TwistLabeled.h>
#include <state_controller/PoseLabeled.h>
#include <state_controller/BehaviorLib.h>

class Teleop{

public:
  explicit Teleop();

private:
  ros::NodeHandle n;
  ros::Subscriber joystick_sub;
  ros::Publisher activate_pub;
  ros::Publisher userinput_pub;
  ros::Publisher drivemsg_pub;
  ros::Publisher hitchmsg_pub;
  ros::Publisher softestop_pub;
  ros::Publisher state_pub;
  std_msgs::Bool stop_msg;
  std_msgs::Bool activate_msg;
  Behavior curr_behavior;
  state_controller::TwistLabeled drive_msg;
  state_controller::PoseLabeled hitch_msg;
  std::vector<Behavior> behaviors;

  void joyCB(const sensor_msgs::Joy::ConstPtr &joy);
  void softestop(bool stop);
  void activate(bool aut);
  void state(Behavior behavior);
  int incrementState(float dir);
  int sendUserInput();
  float computeZPosition(float up_axis, float down_axis);
  float computeYOrientation(int up_button, int down_button);
  std::string controllerType;
  bool estop;
  bool isActivated;

  // Buttons & Axes
  int activateButton;
  int estopButton;
  int userInputButton;
  int behaviorAxis;

  // Button & Axis Flags
  bool estopButtonFlag;
  bool activateButtonFlag;
  bool userInputButtonFlag;
  bool behaviorAxisFlag;

  float priorHitchPositionZ = 0;
  float priorHitchOrientationY = 0;

};

#endif //TELEOP_H
