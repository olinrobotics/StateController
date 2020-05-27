/*
 * @file Teleop.cpp
 * @brief Node for sending joystick commands to state controller
 *
 * This contains functions for parsing raw joystick output based on
 * the class of joystick as set in the rosparam controllerType and
 * publishing parsed output to the main state controller
 * @author Carl Moser
 * @maintainer Olin GRAVL
 * @email olingravl@gmail.com
 * @version 1.3.0
 */

#include "Teleop.h"
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <state_controller/BehaviorLib.h>

/*
 * @brief Advertises & subscribes topics, loads joystick settings
 *
 * This constructor has two default args that specify the gamepad -
 * One explicitly when loading the rosparam, another implicitly when
 * initializing the button and axis attributes.
 * TODO(connor@students): Error for improper pass of controller, remove
 * double-default
 * TODO(connor@students): Handle namespaces better
 */
Teleop::Teleop()
: n("~")
, joystickSub(n.subscribe("/joy", 10, &Teleop::joyCB, this))
, activatePub(n.advertise<std_msgs::Bool>("/state_controller/cmd_activate", 1))
, driveMsgPub(n.advertise<state_controller::TwistLabeled>("/state_controller/cmd_behavior_twist", 1))
, hitchMsgPub(n.advertise<state_controller::PoseLabeled>("/state_controller/cmd_behavior_hitch", 1))
, softEstopPub(n.advertise<std_msgs::Bool>("/softestop", 1))
, statePub(n.advertise<std_msgs::String>("/state_controller/cmd_state", 1))
, userInputPub(n.advertise<std_msgs::String>("/user_input", 1))
, estop(false)
, isActivated(false)
, activateButton(0)
, estopButton(1)
, userInputButton(3)
, behaviorAxis(7)
, estopButtonFlag(false)
, activateButtonFlag(false)
, behaviorAxisFlag(false)
, currBehavior("empty", 2)
{
  // Init and sort behavior list, current behavior
  getBehaviorVector(n, behaviors);
  sort(behaviors.begin(), behaviors.end());
  try {
    currBehavior = behaviors.at(0);
  } catch (const std::exception& e) {
    ROS_ERROR("Behaviors not populated - have you loaded behaviors from your .yaml file?");
    ros::shutdown();
  }

  driveMsg.label = std_msgs::String();
  driveMsg.label.data = "teleop";

  hitchMsg.label = std_msgs::String();
  hitchMsg.label.data = "teleop";

  n.param<std::string>("controllerType", controllerType, "gamepad");
  if (controllerType == "gamepad"){
    activateButton = 0;
    estopButton = 1;
    userInputButton = 3;
    behaviorAxis = 7;
  }
  if (controllerType == "joystick"){
    activateButton = 6;
    estopButton = 0;
    userInputButton = 3;
    behaviorAxis = 7;
  }
}

void Teleop::joyCB(const sensor_msgs::Joy::ConstPtr &joy){
  /*
   * @brief Callback for /joy topic
   *
   * Runs upon each receipt of msg from /joy. Activates, disactivates,
   and estops based on button inputs. Changes state based on axis inputs.
   Passes through Twist messages based on joystick inputs unless estopped
   or disactivated, or message is same as message stored in driveMsg attr.
   *
   * @param[in] joy Message read from /joy topic
   */

  if (joy->axes[5] == 0 || joy->axes[2] == 0) {
    ROS_INFO_ONCE("Hitch has not been initialized. Please pull both triggers to initialize hitch controls");
  } else if (joy->axes[5] != 0 && joy->axes[2] != 0) {
    ROS_INFO_ONCE("Hitch successfully initialized. Hitch controls active");
  }

  //check for estop
  if(joy->buttons[estopButton] && !estop && !estopButtonFlag){
    activate(false);
    softEstop(true);
    estopButtonFlag = true;
    return;
  }
  //check for estop button release
  if(!joy->buttons[estopButton] && !estop && estopButtonFlag){
    isActivated = false;
    estop = true;
    estopButtonFlag = false;
  }
  //check for un-estop
  if(joy->buttons[estopButton] && estop && !estopButtonFlag){
    softEstop(false);
    estopButtonFlag = true;
  }
  //check for un-estop button release
  if(!joy->buttons[estopButton] && estop && estopButtonFlag){
    estop = false;
    estopButtonFlag = false;
  }
  //check for user input button press
  if(joy->buttons[userInputButton] && !userInputButtonFlag){
    sendUserInput();
    userInputButtonFlag = true;
  }
  //check for user input button release
  if(!joy->buttons[userInputButton] && userInputButtonFlag){
    userInputButtonFlag = false;
  }

  //check if not currently estopped
  if(!stopMsg.data){

    //check for statechange
    if(joy->axes[behaviorAxis] && !behaviorAxisFlag){
      incrementState(joy->axes[behaviorAxis]);
      behaviorAxisFlag = true;
    }
    //check for statechange axis release
    if(joy->axes[behaviorAxis] && behaviorAxisFlag){
      behaviorAxisFlag = false;
    }

    //check for activated
    if(joy->buttons[activateButton] && !isActivated && !activateButtonFlag){
      activate(true);
      activateButtonFlag = true;
    }
    //check for activated button release
    if(!joy->buttons[activateButton] && !isActivated && activateButtonFlag){
      isActivated = true;
      activateButtonFlag = false;
    }
    //check for disactivated
    if(joy->buttons[activateButton] && isActivated && !activateButtonFlag){
      activate(false);
      activateButtonFlag = true;
    }
    //check for disactivated button release
    if(!joy->buttons[activateButton] && isActivated && activateButtonFlag){
      isActivated = false;
      activateButtonFlag = false;
    }

    //check if tractor is activated
    if (isActivated){

      // generate and send twist message if unique
      if (driveMsg.twist.angular.z != joy->axes[0]
       || driveMsg.twist.linear.x != joy->axes[1]) {
        driveMsg.twist.angular.z = joy->axes[0];
        driveMsg.twist.linear.x =  joy->axes[1];
        driveMsgPub.publish(driveMsg);
      }
      // if triggers haven't been moved from their default position, do nothing
      // Note: The Joy node assumes all axes default to 0 and will publish
      // 0s for the two triggers until they are pulled. The trigger's "resting"
      // state is 1, not 0, so this becomes problematic and causes "runaway"
      // behavior when only one of the triggers have been pulled. To get around
      // this, the operator must pull both triggers to update the joy node
      // before the hitch will be activated
      // TODO: Fix this bug
      if (joy->axes[5] == 0 || joy->axes[2] == 0) { }
      // generate and send hitch message if unique
      else if (hitchMsg.pose.position.z != computeZPosition(joy->axes[5], joy->axes[2]) ||
               hitchMsg.pose.orientation.y != computeYOrientation(joy->buttons[5], joy->buttons[4])) {
        hitchMsg.pose.position.z = computeZPosition(joy->axes[5], joy->axes[2]);
        hitchMsg.pose.orientation.y = computeYOrientation(joy->buttons[5], joy->buttons[4]);
        hitchMsgPub.publish(hitchMsg);

      }
    }
  }
}

float Teleop::computeZPosition(float up_axis, float down_axis) {
  /*
   * @brief computes new Z position based on trigger axes
   * @param[in] up_axis = right trigger value
                down_axis = left trigger value
   */

   // TODO: Add limits and raise ros warning if they're hit
   if (up_axis < 1 && down_axis < 1) {
     // If both axes are pressed, do nothing
     return priorHitchPositionZ;
   } else if (up_axis < 1) {
     // Increment height by 0.1
     priorHitchPositionZ = priorHitchPositionZ + 0.001;
     return priorHitchPositionZ;
   } else if (down_axis < 1){
     // Decrement height by 0.1
     priorHitchPositionZ = priorHitchPositionZ - 0.001;
     return priorHitchPositionZ;
   } else{
     // If neither are pressed, do nothing
     return priorHitchPositionZ;
   }
}

 float Teleop::computeYOrientation(int up_button, int down_button) {
  /*
   * @brief computes new Y orientation based on trigger buttons
   * @param[in] up_button = right trigger button value
                down_button = left trigger button value
   */
   if (up_button == 1 && down_button ==  1) {
     // If both buttons are pressed, do nothing
     return priorHitchOrientationY;
   } else if (up_button == 1) {
     // Increment angle by 0.1
     priorHitchOrientationY = priorHitchOrientationY + 0.0005;
     return priorHitchOrientationY;
   } else if (down_button == 1) {
     // Decrement angle by 0.1
     priorHitchOrientationY = priorHitchOrientationY - 0.0005;
     return priorHitchOrientationY;
   } else {
     // If neither buttons are pressed, do nothing
     return priorHitchOrientationY;
   }
}

void Teleop::softEstop(bool stop){
  /*
   * @brief publishes software estop command
   * @param[in] stop State of the softestop to publish
   */
  stopMsg.data = stop;
  softEstopPub.publish(stopMsg);
}

void Teleop::activate(bool act){
  /*
   * @brief publishes activation command
   * @param[in] act State of the activate function to publish
   */
  std_msgs::Bool msg;
  msg.data = act;
  activatePub.publish(msg);
}

void Teleop::state(Behavior behavior){
   /*
    * @brief publishes behavior statechange command
    * @param[in] behavior state to publish
    * @param[out] updates currBehavior w/ behavior arg
    */
   currBehavior = behavior;
   statePub.publish(behavior.getLabel());
 }

int Teleop::incrementState(float dir) {
  /*
   * @brief increments state in given direction
   *
   * Updates state to next numeric state or previous numeric state.
   * Implements wrapping of states, so incrementing down from 0th
   * state updates to nth state.
   * TODO(connor@students): Remove hardcoded number of states
   *
   * @param[in] direction to increment based on sign
   * return integer error code 2, 1, 0
   */
  int s = find(behaviors.begin(), behaviors.end(), currBehavior) - behaviors.begin();
  if(s >= behaviors.size()) return 2;

  // If incrementing
  if(dir > 0) state(behaviors[(s + 1) % behaviors.size()]);

  // If decrementing (wrap to remove vals < 0)
  else if (dir < 0) {
    if (s==0) s = behaviors.size();
    state(behaviors[(s - 1)]);
  } else return 1;
  return 0;
}

int Teleop::sendUserInput() {
   // Publish 'y' on /user_input topic
   std_msgs::String msg;
   msg.data = 'y';
   userInputPub.publish(msg);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "teleop");
  Teleop t;
  ros::spin();
}
