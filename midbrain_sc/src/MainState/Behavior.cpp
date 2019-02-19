/*
 * @file Behavior.cpp
 * @brief behavior class correlating names, ids, priorities
 *
 * @author Connor Novak
 * @email connor@students.olin.edu
 * @date 2018-11-21
 * @version 1.0.0
 fix thi thing that kevin broke
 */

#include "Behavior.h"

/*
 * @brief Constructor TODO(connor@students) Turn into struct
 */
Behavior::Behavior(const char* name, const int label)
 : name(name), id(label) { }

int Behavior::getId() {
  return id;
}

midbrain_sc::TwistLabeled Behavior::getTwistMessage() {
  return twist_message;
}

midbrain_sc::ArrayLabeled Behavior::getArrayMessage() {
  return array_message;
}


void Behavior::setMessage(midbrain_sc::TwistLabeled msg) {
  //TODO(connor@students): Check that label is equal to id
  twist_message = msg;
}

void Behavior::setMessage(midbrain_sc::ArrayLabeled msg) {
  //TODO(connor@students): Check that label is equal to id
  array_message = msg;
}
