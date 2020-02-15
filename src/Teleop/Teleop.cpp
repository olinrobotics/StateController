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
, joystick_sub(n.subscribe("/joy", 10, &Teleop::joyCB, this))
, activate_pub(n.advertise<std_msgs::Bool>("/state_controller/cmd_activate", 1))
, drivemsg_pub(n.advertise<state_controller::TwistLabeled>("/state_controller/cmd_behavior_twist", 1))
, hitchmsg_pub(n.advertise<state_controller::PoseLabeled>("/state_controller/cmd_behavior_hitch", 1))
, softestop_pub(n.advertise<std_msgs::Bool>("/softestop", 1))
, state_pub(n.advertise<std_msgs::String>("/state_controller/cmd_state", 1))
, userinput_pub(n.advertise<std_msgs::String>("/user_input", 1))
, estop(false)
, isActivated(false)
, activateButton(0)
, estopButton(1)
, userInputButton(3)
, behaviorAxis(7)
, estopButtonFlag(false)
, activateButtonFlag(false)
, behaviorAxisFlag(false)
, curr_behavior("empty", 2)
{
  // Init and sort behavior list, current behavior
  getBehaviorVector(n, behaviors);
  sort(behaviors.begin(), behaviors.end());
  try {
    curr_behavior = behaviors.at(0);
  } catch (const std::exception& e) {
    ROS_ERROR("Behaviors not populated - have you loaded behaviors from your .yaml file?");
    ros::shutdown();
  }

  drive_msg.label = std_msgs::String();
  drive_msg.label.data = "teleop";

  hitch_msg.label = std_msgs::String();
  hitch_msg.label.data = "teleop";

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
   or disactivated, or message is same as message stored in drive_msg attr.
   *
   * @param[in] joy Message read from /joy topic
   */
  //check for estop
  if(joy->buttons[estopButton] && !estop && !estopButtonFlag){
    activate(false);
    softestop(true);
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
    softestop(false);
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
  if(!stop_msg.data){

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
      if (drive_msg.twist.angular.z != joy->axes[0]
       || drive_msg.twist.linear.x != joy->axes[1]) {
        drive_msg.twist.angular.z = joy->axes[0];
        drive_msg.twist.linear.x =  joy->axes[1];
        drivemsg_pub.publish(drive_msg);
      }
      // // generate and send hitch message
      if (hitch_msg.pose.position.z != computeZPosition(joy->axes[5], joy->axes[2]) ||
          hitch_msg.pose.orientation.y != computeYOrientation(joy->buttons[5], joy->buttons[4])) {
        hitch_msg.pose.position.z = computeZPosition(joy->axes[5], joy->axes[2]);
        hitch_msg.pose.orientation.y = computeYOrientation(joy->buttons[5], joy->buttons[4]);
        hitchmsg_pub.publish(hitch_msg);

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
     priorHitchPositionZ = priorHitchPositionZ + 0.0005;
     return priorHitchPositionZ;
   } else if (down_axis < 1){
     // Decrement height by 0.1
     priorHitchPositionZ = priorHitchPositionZ - 0.0005;
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

void Teleop::softestop(bool stop){
  /*
   * @brief publishes software estop command
   * @param[in] stop State of the softestop to publish
   */
  stop_msg.data = stop;
  softestop_pub.publish(stop_msg);
}

void Teleop::activate(bool act){
  /*
   * @brief publishes activation command
   * @param[in] act State of the activate function to publish
   */
  activate_msg.data = act;
  activate_pub.publish(activate_msg);
}

void Teleop::state(Behavior behavior){
   /*
    * @brief publishes behavior statechange command
    * @param[in] behavior state to publish
    * @param[out] updates curr_behavior w/ behavior arg
    */
   curr_behavior = behavior;
   state_pub.publish(behavior.getLabel());
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
  int s = find(behaviors.begin(), behaviors.end(), curr_behavior) - behaviors.begin();
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
   userinput_pub.publish(msg);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "teleop");
  Teleop t;
  ros::spin();
}
