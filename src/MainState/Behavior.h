/*
 * @file Behavior.h
 * @brief behavior function prototypes
 *
 * @author Connor Novak
 * @email connor@students.olin.edu
 */

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <state_controller/TwistLabeled.h>
#include <state_controller/ArrayLabeled.h>
#include <string>

class Behavior {
  public:
    Behavior(std_msgs::String label, const int priority);

    // Getters and Setters
    int getPriority();
    std_msgs::String getLabel();
    state_controller::TwistLabeled getTwistMessage();
    state_controller::ArrayLabeled getArrayMessage();
    void setMessage(state_controller::TwistLabeled msg);
    void setMessage(state_controller::ArrayLabeled msg);

  private:
    int priority;
    std_msgs::String label;
    state_controller::TwistLabeled twist_message;
    state_controller::ArrayLabeled array_message;

};

#endif //BEHAVIOR_H
