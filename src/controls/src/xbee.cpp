#include "../include/xbee/xbee.hpp"

int xbee::createTrama(char addr1,  char addr0,  char data1,  char data2,  char options) {
  trama[0] = 0x7E;      //start delimiter
  trama[1] = 0x00;      //length
  trama[2] = 0x07;      //length
  trama[3] = 0x01;      //frame type
  trama[4] = 0x00;      //frame ID   ->0x00 no confirmation
  trama[5] = addr1;
  trama[6] = addr0;
  trama[7] = options;
  trama[8] = data1;
  trama[9] = data2;

  int pos = 8;
  char sum = 0x00;
  for(int i = 3; i < 10; i++)
    sum += trama[i];

  char checksum = (0xFF - sum);

  if((data1 == 0x7E) || (data1 == 0x7D) || (data1 == 0x11) || (data1 == 0x13)) {
    trama[pos] = 0x7D;  //escape byte
    pos++;
    trama[pos] = data1 ^ 0x20;
    pos++;
  }
  else {
    trama[pos] = data1;
    pos++;
  }

  if((data2 == 0x7E) || (data2 == 0x7D) || (data2 == 0x11) || (data2 == 0x13)) {
    trama[pos] = 0x7D; //escape byte
    pos++;
    trama[pos] = data2 ^ 0x20;
    pos++;
  }
  else {
    trama[pos] = data2;
    pos++;
  }

  if((checksum == 0x7E) || (checksum == 0x7D) || (checksum == 0x11) || (checksum == 0x13)) {
    trama[pos] = 0x7D;     //escape byte
    pos++;
    trama[pos] = checksum ^ 0x20;
    pos++;
  }
  else {
    trama[pos] = checksum;
    pos++;
  }

  return pos;
}

void xbee::velJoystickA(const geometry_msgs::Twist speed) {

  cout << "VEL Linear:"  << speed.linear.x	<< endl; // 127 a -127; frente e trás
  cout << "VEL Angular:" << speed.angular.z	<< endl; // 127 a -127; esquerda e direitta

  int vel1 = speed.linear.x/3;
  int vel2 = speed.angular.z/2;

  if(speed.angular.z == 2) {
    vel1 = vel1 * 1.6;
    vel2 = vel2 * 1.6;
  }

  int turn_control_1 = -vel2 + vel2/2 + vel2/3;
  int turn_control_2 = vel2 - vel2/2 - vel2/3;
  int vel_control_1 = vel1 - vel1/3 - vel2 + vel2/2 + vel2/4 + vel2/6;
  int vel_control_2 = vel1 - vel1/3 + vel2 - vel2/2 - vel2/4 - vel2/6;

  int sizeTrama = 0;
  if(vel1 < 5 && vel1 > -5)
    sizeTrama = createTrama(addr0RobotA, addr1RobotA, (0x3F + turn_control_1), (0x3F + turn_control_2), 0x01);
  else {
    if((0x7E + vel_control_1) > 255)
      vel1  = 255;
    else if((0x7E + vel_control_1 ) < 0)
      vel1  = 0;
    if((0x7E + vel_control_2) > 255)
      vel2  = 255;
    else if((0x7E + vel_control_2) < 0)
      vel2  = 0;

    sizeTrama = createTrama(addr0RobotA, addr1RobotA, (0x3F + vel_control_1), (0x3F + vel_control_2), 0x01);
  }

  cout << "SENDING: ";
  for(int i = 0; i < sizeTrama; i++)
    cout << hex << (unsigned int)trama[i] << "_";
  cout << endl;

  my_serial_stream.write((char*)trama, sizeTrama);
}

void xbee::velJoystickB(const geometry_msgs::Twist speed) {

  cout << "VEL Linear:"  << speed.linear.x	<< endl; // 127 a -127; frente e trás
  cout << "VEL Angular:" << speed.angular.z	<< endl; // 127 a -127; esquerda e direitta

  int vel1 = speed.linear.x/3;
  int vel2 = speed.angular.z/2;

  if(speed.angular.z == 2) {
    vel1 = vel1 * 1.6;
    vel2 = vel2 * 1.6;
  }

  int turn_control_1 = -vel2 + vel2/2 + vel2/3;
  int turn_control_2 = vel2 - vel2/2 - vel2/3;
  int vel_control_1 = vel1 - vel1/3 - vel2 + vel2/2 + vel2/4 + vel2/6;
  int vel_control_2 = vel1 - vel1/3 + vel2 - vel2/2 - vel2/4 - vel2/6;

  int sizeTrama = 0;
  if(vel1 < 5 && vel1 > -5)
    sizeTrama = createTrama(addr0RobotB, addr1RobotB, (0x3F + turn_control_1), (0x3F + turn_control_2), 0x01);
  else {
    if((0x7E + vel_control_1) > 255)
      vel1  = 255;
    else if((0x7E + vel_control_1 ) < 0)
      vel1  = 0;
    if((0x7E + vel_control_2) > 255)
      vel2  = 255;
    else if((0x7E + vel_control_2) < 0)
      vel2  = 0;

    sizeTrama = createTrama(addr0RobotB, addr1RobotB, (0x3F + vel_control_1), (0x3F + vel_control_2), 0x01);
  }

  cout << "SENDING: ";
  for(int i = 0; i < sizeTrama; i++)
    cout << hex << (unsigned int)trama[i] << "_";
  cout << endl;

  my_serial_stream.write((char*)trama, sizeTrama);
}

void xbee::velJoystickC(const geometry_msgs::Twist speed) {

  cout << "VEL Linear:"  << speed.linear.x	<< endl; // 127 a -127; frente e trás
  cout << "VEL Angular:" << speed.angular.z	<< endl; // 127 a -127; esquerda e direitta

  int vel1 = speed.linear.x/3;
  int vel2 = speed.angular.z/2;

  if(speed.angular.z == 2) {
    vel1 = vel1 * 1.6;
    vel2 = vel2 * 1.6;
  }

  int turn_control_1 = -vel2 + vel2/2 + vel2/3;
  int turn_control_2 = vel2 - vel2/2 - vel2/3;
  int vel_control_1 = vel1 - vel1/3 - vel2 + vel2/2 + vel2/4 + vel2/6;
  int vel_control_2 = vel1 - vel1/3 + vel2 - vel2/2 - vel2/4 - vel2/6;

  int sizeTrama = 0;
  if(vel1 < 5 && vel1 > -5)
    sizeTrama = createTrama(addr0RobotC, addr1RobotC, (0x3F + turn_control_1), (0x3F + turn_control_2), 0x01);
  else {
    if((0x7E + vel_control_1) > 255)
      vel1  = 255;
    else if((0x7E + vel_control_1 ) < 0)
      vel1  = 0;
    if((0x7E + vel_control_2) > 255)
      vel2  = 255;
    else if((0x7E + vel_control_2) < 0)
      vel2  = 0;

    sizeTrama = createTrama(addr0RobotC, addr1RobotC, (0x3F + vel_control_1), (0x3F + vel_control_2), 0x01);
  }

  cout << "SENDING: ";
  for(int i = 0; i < sizeTrama; i++)
    cout << hex << (unsigned int)trama[i] << "_";
  cout << endl;

  my_serial_stream.write((char*)trama, sizeTrama);
}

void xbee::velJoystickD(const geometry_msgs::Twist speed) {

  cout << "VEL Linear:"  << speed.linear.x	<< endl; // 127 a -127; frente e trás
  cout << "VEL Angular:" << speed.angular.z	<< endl; // 127 a -127; esquerda e direitta

  int vel1 = speed.linear.x/3;
  int vel2 = speed.angular.z/2;

  if(speed.angular.z == 2) {
    vel1 = vel1 * 1.6;
    vel2 = vel2 * 1.6;
  }

  int turn_control_1 = -vel2 + vel2/2 + vel2/3;
  int turn_control_2 = vel2 - vel2/2 - vel2/3;
  int vel_control_1 = vel1 - vel1/3 - vel2 + vel2/2 + vel2/4 + vel2/6;
  int vel_control_2 = vel1 - vel1/3 + vel2 - vel2/2 - vel2/4 - vel2/6;

  int sizeTrama = 0;
  if(vel1 < 5 && vel1 > -5)
    sizeTrama = createTrama(addr0RobotD, addr1RobotD, (0x3F + turn_control_1), (0x3F + turn_control_2), 0x01);
  else {
    if((0x7E + vel_control_1) > 255)
      vel1  = 255;
    else if((0x7E + vel_control_1 ) < 0)
      vel1  = 0;
    if((0x7E + vel_control_2) > 255)
      vel2  = 255;
    else if((0x7E + vel_control_2) < 0)
      vel2  = 0;

    sizeTrama = createTrama(addr0RobotD, addr1RobotD, (0x3F + vel_control_1), (0x3F + vel_control_2), 0x01);
  }

  cout << "SENDING: ";
  for(int i = 0; i < sizeTrama; i++)
    cout << hex << (unsigned int)trama[i] << "_";
  cout << endl;

  my_serial_stream.write((char*)trama, sizeTrama);
}
