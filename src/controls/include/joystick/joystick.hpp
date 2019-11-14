#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "game_engine/ShootAndTurbo.h"
#include <iostream>
#include <sstream>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <libusb-1.0/libusb.h>
#include "../../../game_engine/include/utils/system_info.hpp"

#define number_joysticks 4

uint16_t VENDOR_A  = 0x0079;
uint16_t PRODUCT_A = 0x0006;
uint16_t VENDOR_B  = 0x2563;
uint16_t PRODUCT_B = 0x0523;

using namespace std;

geometry_msgs::Twist CmdVel[number_joysticks];
game_engine::ShootAndTurbo shoot[number_joysticks];
ros::NodeHandle* NodeHandle;
ros::Publisher CmdVel_pub[number_joysticks], shoot_pub[number_joysticks];
pthread_t joystick[number_joysticks];
pthread_mutex_t mutex[number_joysticks];
libusb_context *ctx = NULL;

/*!
 * - This structure contains each joystick information
 */

struct joystickHandler {
  libusb_device *dev;
  libusb_device_handle *handle;
  uint16_t product_id;
  uint16_t vendor_id;
};

joystickHandler *joy;

/*!
 * - Joystick class, used to publish controls over ROS
 */

class controller {
public:
  /*! - class destructor, exits libusb */
  ~controller();

  /*! - publishes joystick A controls on cmd_velA topic */
  static void* joystickA(void *arg);
  /*! - publishes joystick B controls on cmd_velB topic */
  static void* joystickB(void *arg);
  /*! - publishes joystick C controls on cmd_velC topic */
  static void* joystickC(void *arg);
  /*! - publishes joystick D controls on cmd_velD topic */
  static void* joystickD(void *arg);
};

void *(*joyHandlers[4])(void *) = {controller::joystickA, controller::joystickB, controller::joystickC, controller::joystickD};

#endif // JOYSTICK_HPP
