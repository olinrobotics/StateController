// References https://answers.ros.org/question/201977/include-header-file-from-another-package-indigo/
#pragma once

#include <vector>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <state_controller/TwistLabeled.h>
#include <state_controller/ArrayLabeled.h>

class Behavior {
  public:
    Behavior(std_msgs::String label, const int priority);
    Behavior(std::string label_raw, const int priority);

    // Getters and Setters
    int getPriority();
    std_msgs::String getLabel();
    state_controller::TwistLabeled getTwistMessage();
    state_controller::ArrayLabeled getArrayMessage();
    void setMessage(state_controller::TwistLabeled msg);
    void setMessage(state_controller::ArrayLabeled msg);

    // Overloads
    bool operator==(Behavior b);
    // bool operator< (Behavior b);
    // bool operator> (Behavior b);
    // bool operator>=(Behavior b);
    // bool operator<=(Behavior b);

  private:

    // Data Attributes
    int priority;
    std_msgs::String label;
    state_controller::TwistLabeled twist_message;
    state_controller::ArrayLabeled array_message;

};

std::vector<Behavior> getBehaviors(ros::NodeHandle n);
bool compareBehaviorPriority(Behavior b1, Behavior b2);
