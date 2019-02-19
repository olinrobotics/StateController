/*
 * @file Behavior.h
 * @brief behavior function prototypes
 *
 * @author Connor Novak
 * @email connor@students.olin.edu
 */

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "midbrain_sc/TwistLabeled.h"
#include "midbrain_sc/ArrayLabeled.h"


class Behavior {
  public:
    Behavior(const char* n, const int l);

  // Getters and Setters
    int getId();
    midbrain_sc::TwistLabeled getTwistMessage();
    midbrain_sc::ArrayLabeled getArrayMessage();
    void setMessage(midbrain_sc::TwistLabeled msg);
    void setMessage(midbrain_sc::ArrayLabeled msg);

  private:
    int id;
    const char* name;
    midbrain_sc::TwistLabeled twist_message;
    midbrain_sc::ArrayLabeled array_message;

};

#endif //BEHAVIOR_H
