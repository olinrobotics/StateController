/*
 * @file Behavior.h
 * @brief behavior function prototypes
 *
 * @author Connor Novak
 * @email connor@students.olin.edu
 */

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <midbrain_sc/TwistLabeled.h>
#include <midbrain_sc/ArrayLabeled.h>
#include <string>

class Behavior {
  public:
    Behavior(std_msgs::String label, const int priority);

    // Getters and Setters
    int getPriority();
    std_msgs::String getLabel();
    midbrain_sc::TwistLabeled getTwistMessage();
    midbrain_sc::ArrayLabeled getArrayMessage();
    void setMessage(midbrain_sc::TwistLabeled msg);
    void setMessage(midbrain_sc::ArrayLabeled msg);

  private:
    int priority;
    std_msgs::String label;
    midbrain_sc::TwistLabeled twist_message;
    midbrain_sc::ArrayLabeled array_message;

};

#endif //BEHAVIOR_H
