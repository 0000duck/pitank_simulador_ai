#ifndef XBEE_HPP
#define XBEE_HPP

#include </usr/include/SerialStream.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#define addr0RobotA 0x10
#define addr1RobotA 0x01
#define addr0RobotB 0x10
#define addr1RobotB 0x01
#define addr0RobotC 0x10
#define addr1RobotC 0x03
#define addr0RobotD 0x10
#define addr1RobotD 0x04

using namespace std;
using namespace LibSerial;

class xbee {
public:
  SerialStream my_serial_stream, my_serial_port;

  void velJoystickA(geometry_msgs::Twist speed);
  void velJoystickB(geometry_msgs::Twist speed);
  void velJoystickC(geometry_msgs::Twist speed);
  void velJoystickD(geometry_msgs::Twist speed);
private:
  char trama[15];

  int createTrama(char addr0, char addr1, char data1, char data2, char options);
};

#endif // XBEE_HPP
