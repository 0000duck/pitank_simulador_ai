#include "../include/xbee/xbee.hpp"

volatile int signal_flag;
void sig_handler(int signal) {
  signal_flag = 0;
}

int main(int argc, char** argv) {
  //rosrun controls xbee /dev/ttyUSB0

  if(argc < 2 || argv[1] == NULL) {
    cout << "Error running the program" << endl;
    return 1;
  }

  xbee xbeeObj;

  string s;
  s.append("sudo chmod 777 ");
  s.append(argv[1]);
  int command = system(s.c_str());
  signal(SIGINT, sig_handler);

  xbeeObj.my_serial_stream.Open(argv[1]);
  xbeeObj.my_serial_stream.SetBaudRate(SerialStreamBuf::BAUD_57600);
  xbeeObj.my_serial_port.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
  xbeeObj.my_serial_port.SetNumOfStopBits(1);
  xbeeObj.my_serial_port.SetParity(SerialStreamBuf::PARITY_NONE);
  xbeeObj.my_serial_port.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);

  if(xbeeObj.my_serial_stream.good()) {
    cout << "SUCCESSFUL: serial port opened at: /dev/ttyUSB0" << endl;
    usleep(5000);
  }
  else {
    cout << "ERROR: Could not open serial port." << endl;
    return 1;
  }

  ros::init(argc, argv, "Xbee");

  while(1) {
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cmd_velA", 1, &xbee::velJoystickA, &xbeeObj);
    ros::spin();
  }
  return 0;
}
